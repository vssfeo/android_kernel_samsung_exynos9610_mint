/*
 * hdk_sensor_core.ko — прямой realtime-доступ к датчикам из ядра (LOGGER).
 *
 * ЗАМЕНЯЕТ СРАЗУ ДВА КОМПОНЕНТА: демон hdk_iiod и подмену hals.conf.
 * Модуль становится единственным владельцем IIO-буферов сенсоров
 * (/dev/iio:device*, которые SSP-драйвер включает через ssp_sensor/enable),
 * читает их realtime-потоками ядра и отдаёт кадры приложению через
 * mmap-кольцо без единого syscall на кадр.
 *
 * ПОЧЕМУ ЭТО ЛУЧШЕ ЗА «ПОДМЕНУ hals.conf»
 *   Нативный HAL (sensors.sensorhub.so) при включённом прямом режиме держит
 *   те же буферы — тогда демон получает «Device or resource busy» и 0 кадров
 *   (проверено на стенде, docs/SENSOR_DIRECT_KERNEL.md §5). Модуль занимает
 *   буферы сам; HAL, не имея к ним доступа, отдаёт нулевое перечисление —
 *   то есть «службы отключены» не подменой конфига, а физическим владением.
 *
 * САМОВОССТАНОВЛЕНИЕ — главная ценность модуля
 *   Преребутовое состояние стенда: хаб отвечает (mcu_test OK), но
 *   sensor_probe_state в драйвере пуст, поэтому enable_sensor() возвращает
 *   -ENODEV, ещё НЕ отправив команду в хаб; sensorservice показывает
 *   «No Sensors on the device», демон — 0 Гц. Из userspace это не лечится
 *   (проверено: mcu_test, перевключение enable, ssp_flush — без эффекта).
 *   Модуоль может вызвать то, что доступно только из ядра:
 *     get_sensor_scanning_info() — перечитать sensor_probe_state у хаба;
 *     sensorhub_reset()          — переподключить хаб (sysfs mcu_reset
 *                                   запрещён: атрибут -r--r--r--);
 *     set_delay_legacy_sensor()  — задать частоту опроса напрямую.
 *   Порядок восстановления: disable всех -> (если хаб не жив) reset ->
 *   перечитать scanning -> enable + задать ODR -> открыть каналы заново.
 *
 * ПРИОРИТЕТ
 *   kthread'ы: SCHED_FIFO 98, привязка к отдельным ядрам, никакого
 *   userspace-планировщика. Данные идут из буфера в кольцо в контексте ядра.
 *
 * АБСТРАКЦИЯ КОЛЬЦА
 *   Кольцо выделено vzalloc (постранично) и отдаётся приложению через
 *   remap_vmalloc_range — это безопасный штатный способ: virt_to_phys на
 *   kmalloc-память в remap_pfn_range дал бы BUG, память не выровнена по
 *   страницам.
 *
 * ФОРМАТ КАДРА — совместим с sensor_direct.c (24 Б, little-endian):
 *   int64 t_ns; uint16 ch; uint16 n; int16 v[6];
 *   ch: 0=acc, 3=gyro+bias, 4=mag+bias (индексы как в hdk_iiod).
 *
 * ЖИЗНЕННЫЙ ЦИКЛ
 *   init:  найти драйвер -> разредить символы -> зарегистрировать misc
 *         -> включить датчики и открыть каналы -> поднять потоки.
 *   exit:  стоп=1 -> потоки выходят сами (чтение неблокирующее) ->
 *         закрыть файлы -> снять датчики -> deregister.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kallsyms.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/types.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/ktime.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/utsname.h>
#include <linux/cpumask.h>
#include <linux/atomic.h>
/*
 * init_uts_ns намеренно НЕ импортируется: на этом ядре символ не
 * экспортирован, а при CONFIG_MODVERSIONS импорт без экспорта даёт
 * отказ загрузки модуля (ENOEXEC). Версию ядра читаем из userspace.
 */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Helideck LOGGER");
MODULE_DESCRIPTION("hdk_sensor_core: прямой realtime-доступ к датчикам из ядра");
MODULE_VERSION("1.0.0");

/* ============================== ABI =============================== */
#define HDK_MAGIC      0x48444B52u          /* "HDKR" */
#define HDK_VERSION    1u
#define HDK_RING_CAP   8192u               /* кадров, степень двойки */
#define HDK_FRAME_N    5                   /* каналов в ABI (как hdk_iiod) */
#define HDK_DEV_NAME   "hdk_sensors"
#define HDK_READ_BUF   16384u

/* 24 байта — тот же формат, что ждёт sensor_direct.c. */
struct hdk_frame {
    int64_t  t_ns;
    uint16_t ch;
    uint16_t n;
    int16_t  v[6];
} __packed;

struct hdk_ring {
    uint32_t magic;
    uint32_t version;
    uint32_t frame_size;
    uint32_t ring_cap;
    volatile uint64_t prod;
    volatile uint64_t cons;
    volatile uint64_t dropped;
    volatile int64_t  rate_mHz[HDK_FRAME_N];
    volatile uint64_t frames[HDK_FRAME_N];
    /* счётчики восстановления и ошибок открытия */
    volatile uint64_t reenable_count;
    volatile uint64_t open_fail;
};

#define HDK_IOC_MAGIC 'H'
#define HDK_IOCTL_RESET_CHUB   _IO(HDK_IOC_MAGIC, 1)
#define HDK_IOCTL_GET_STATS    _IOR(HDK_IOC_MAGIC, 2, struct hdk_stats)
#define HDK_IOCTL_FORCE_REENROLL _IO(HDK_IOC_MAGIC, 3)
struct hdk_stats {
    uint64_t prod, cons, dropped, reenable_count, open_fail;
    int64_t  rate_mHz[HDK_FRAME_N];
    uint64_t frames[HDK_FRAME_N];
    uint32_t hub_working;
    uint32_t ch_open[3];
    uint32_t driver_ok;
};

/* ===================== символы SSP-драйвера ====================== */
struct ssp_data;

extern unsigned long kallsyms_lookup_name(const char *name);
extern struct class *sensors_class;

static int (*fn_enable)(struct ssp_data *, unsigned int, u8 *, int);
static int (*fn_disable)(struct ssp_data *, unsigned int, u8 *, int);
static int (*fn_set_delay)(struct ssp_data *, unsigned int, int, int);
static int (*fn_get_scanning)(struct ssp_data *);
static int (*fn_hub_reset)(void *);
static bool (*fn_is_working)(void *);
static bool driver_ok;

static struct ssp_data *ssp_data;

/* ======================= каналы ================================== */
enum { CH_ACC = 0, CH_GYRO = 1, CH_MAG = 2, CH_GYRO_B = 3, CH_MAG_B = 4 };

struct hdk_chan {
    const char *name;
    int  ssp_type;     /* тип SSP: 1 acc, 16 uncal_gyro, 14 uncal_mag */
    int  ring_ch;
    int  delay_ms;     /* период опроса */
    /* состояние принадлежит потоку-читателю, не watchdog'у */
    struct file *filp;
    uint8_t *buf;
    int  frame_bytes;
    bool opened;
} CH_CFG[3] = {
    { "/dev/iio:device1", 1,  CH_ACC,    4, NULL, NULL, 0, false },
    { "/dev/iio:device7", 16, CH_GYRO_B, 4, NULL, NULL, 0, false },
    { "/dev/iio:device6", 14, CH_MAG_B,  8, NULL, NULL, 0, false },
};

/* ========================= состояние ============================== */
static struct hdk_ring *ring;
static size_t ring_bytes;
static struct hdk_frame *ring_frames;

static struct task_struct *thr[3];
static atomic_t stop = ATOMIC_INIT(0);
static atomic_t need_reenroll = ATOMIC_INIT(0);
static struct mutex hub_lock;      /* сериализует операции с хабом */
static atomic_t reenable_count = ATOMIC_INIT(0);
static atomic_t open_fail = ATOMIC_INIT(0);
static unsigned long total_resets;

/* Опережающее объявление: вызывается из потока-читателя, определена ниже. */
static void reenroll_channels(void);

/* ---------------- доступ к функциям драйвера ---------------- */
static int resolve_driver(void)
{
    unsigned long a;

#define RES(name, ptr) \
    do { \
        a = kallsyms_lookup_name(name); \
        if (!a) { pr_err("hdk_sc: символ %s НЕ найден\n", name); return -ENOENT; } \
        *(unsigned long *)&(ptr) = a; \
    } while (0)

    RES("enable_sensor", fn_enable);
    RES("disable_sensor", fn_disable);
    RES("set_delay_legacy_sensor", fn_set_delay);
    RES("get_sensor_scanning_info", fn_get_scanning);
    RES("sensorhub_reset", fn_hub_reset);
    RES("is_sensorhub_working", fn_is_working);
#undef RES
    return 0;
}

static int match_ssp_sensor(struct device *dev, const void *data)
{
    return strcmp(dev_name(dev), (const char *)data) == 0;
}

static struct ssp_data *find_ssp_data(void)
{
    struct device *dev;
    struct ssp_data *d;

    dev = class_find_device(sensors_class, NULL, "ssp_sensor", match_ssp_sensor);
    if (!dev)
        return NULL;
    d = dev_get_drvdata(dev);
    put_device(dev);
    return d;
}

static int enable_type(int type, int on)
{
    u8 buf[2] = {0, 0};
    if (!driver_ok || !ssp_data)
        return -ENODEV;
    if (on)
        return fn_enable(ssp_data, type, buf, sizeof(buf));
    return fn_disable(ssp_data, type, buf, sizeof(buf));
}

/* Размер кадра IIO — из sysfs драйвера (источник истины, как в hdk_iiod). */
static int chan_frame_bytes(const char *dev)
{
    const char *base = strrchr(dev, '/');
    char path[96], line[64] = {0}, *slash;
    struct file *f;
    loff_t pos = 0;
    int n, bits = 0;

    base = base ? base + 1 : dev;
    snprintf(path, sizeof(path),
             "/sys/bus/iio/devices/%s/scan_elements/in_timestamp_type", base);
    f = filp_open(path, O_RDONLY, 0);
    if (IS_ERR(f))
        return 0;
    n = kernel_read(f, line, sizeof(line) - 1, &pos);
    filp_close(f, NULL);
    if (n <= 0)
        return 0;
    slash = strrchr(line, '/');          /* "le:s112/112>>0" -> 112 бит */
    if (!slash)
        return 0;
    bits = simple_strtol(slash + 1, NULL, 10);
    return bits > 0 ? bits / 8 : 0;
}

/*
 * Открыть/закрыть канал. Вызывается ТОЛЬКО из потока-читателя этого канала:
 * так файл не может быть закрыт под тем, кто в нём спит (use-after-free).
 */
static int chan_open(struct hdk_chan *c)
{
    if (c->opened)
        return 0;
    c->frame_bytes = chan_frame_bytes(c->name);
    if (c->frame_bytes <= 0) {
        pr_warn("hdk_sc: %s: нет scan_elements\n", c->name);
        return -EINVAL;
    }
    if (!c->buf) {
        c->buf = kzalloc(HDK_READ_BUF, GFP_KERNEL);
        if (!c->buf)
            return -ENOMEM;
    }
    /* O_NONBLOCK: чтение не должно блокировать модуль навсегда — иначе
     * rmmod/kthread_stop зависнут, если канал молчит. */
    c->filp = filp_open(c->name, O_RDONLY | O_NONBLOCK, 0);
    if (IS_ERR(c->filp)) {
        pr_warn("hdk_sc: %s: %ld (буфер держит HAL?)\n", c->name,
                PTR_ERR(c->filp));
        c->filp = NULL;
        atomic_inc(&open_fail);
        return -EBUSY;
    }
    c->opened = true;
    pr_info("hdk_sc: %s открыт (кадр %d Б, %d мс)\n",
            c->name, c->frame_bytes, c->delay_ms);
    return 0;
}

static void chan_close(struct hdk_chan *c)
{
    if (c->filp) {
        filp_close(c->filp, NULL);
        c->filp = NULL;
    }
    c->opened = false;
}

/* Разбор буфера IIO в кадры кольца (формат как в sensor_direct.c). */
static void chan_pump(struct hdk_chan *c, ssize_t r)
{
    int frames = (int) r / c->frame_bytes;
    int need = c->frame_bytes;
    int i;

    for (i = 0; i < frames; i++) {
        const uint8_t *p = c->buf + (size_t) i * need;
        struct hdk_frame fr;
        uint64_t prod, slot, ch_idx = (uint64_t)(c - CH_CFG);
        uint64_t t = 0;
        int off, k;

        memset(&fr, 0, sizeof(fr));
        fr.ch = (uint16_t) c->ring_ch;
        fr.n = (uint16_t)((need - 8) / 2);
        if (fr.n > 6)
            fr.n = 6;
        off = 0;
        for (k = 0; k < fr.n; k++) {
            fr.v[k] = (int16_t)(p[off] | (p[off + 1] << 8));
            off += 2;
        }
        off = need - 8;
        for (k = 7; k >= 0; k--)
            t = (t << 8) | p[off + k];
        fr.t_ns = (int64_t) t;

        prod = __atomic_load_n(&ring->prod, __ATOMIC_RELAXED);
        slot = prod & (HDK_RING_CAP - 1);
        if (prod - __atomic_load_n(&ring->cons, __ATOMIC_RELAXED) >= HDK_RING_CAP)
            __atomic_add_fetch(&ring->dropped, 1, __ATOMIC_RELAXED);
        ring_frames[slot] = fr;
        __atomic_store_n(&ring->prod, prod + 1, __ATOMIC_RELEASE);
        __atomic_add_fetch(&ring->frames[ch_idx], 1, __ATOMIC_RELAXED);
    }
}

/* ==================== поток чтения канала ========================= */
static int chan_thread(void *arg)
{
    int idx = (int)(long)arg;
    struct hdk_chan *c = &CH_CFG[idx];

    pr_info("hdk_sc: поток %d (%s) pid %d\n", idx, c->name,
            task_pid_nr(current));

    /* каналы включаем/настраиваем здесь же — файл принадлежит потоку */
    chan_open(c);

    while (!kthread_should_stop() && !atomic_read(&stop)) {

        if (atomic_read(&need_reenroll)) {
            /* Освобождаем буфер ДО переинициализации хаба: иначе хаб не
             * сможет пересоздать поток, пока буфер занят нами. */
            chan_close(c);
            reenroll_channels();
            atomic_set(&need_reenroll, 0);
            chan_open(c);
            continue;
        }

        if (!c->opened || !c->filp) {
            if (chan_open(c) != 0)
                msleep(50);
            continue;
        }

        {
            loff_t pos = 0;
            ssize_t r = kernel_read(c->filp, c->buf, HDK_READ_BUF, &pos);
            if (r > 0) {
                chan_pump(c, r);
            } else if (r == -EAGAIN) {
                /* кадров пока нет: короткая пауза, чтобы rmmod был возможен */
                usleep_range(200, 400);
            } else if (r == -ERESTARTSYS || r == -EINTR) {
                continue;
            } else if (r == -ENODEV || r == -ENXIO) {
                /* канал отвалился (сброс хаба) — ждём восстановления */
                pr_warn("hdk_sc: %s: read %ld\n", c->name, (long)r);
                chan_close(c);
                atomic_set(&need_reenroll, 1);
                msleep(20);
            } else {
                chan_close(c);
                msleep(50);
            }
        }
    }

    chan_close(c);
    pr_info("hdk_sc: поток %d вышел\n", idx);
    return 0;
}

/* ============= переинициализация хаба и каналов ==================
 * Вызывается из watchdog/ioctl. Операции с хабом сериализованы, но ФАЙЛЫ
 * каналов не трогает — ими владеют потоки-читатели. */
static void reenroll_channels(void)
{
    int i, did_reset = 0;

    mutex_lock(&hub_lock);

    for (i = 0; i < 3; i++)
        enable_type(CH_CFG[i].ssp_type, 0);
    msleep(50);

    if (driver_ok && ssp_data && !fn_is_working(ssp_data)) {
        pr_warn("hdk_sc: хаб не отвечает — sensorhub_reset() из ядра\n");
        fn_hub_reset(ssp_data);
        total_resets++;
        did_reset = 1;
        msleep(300);
    }

    /* Перечитать состояние датчиков у хаба — именно пустой
     * sensor_probe_state вызывает -19 в enable_sensor(). */
    if (driver_ok && ssp_data)
        fn_get_scanning(ssp_data);

    for (i = 0; i < 3; i++) {
        struct hdk_chan *c = &CH_CFG[i];
        if (enable_type(c->ssp_type, 1) < 0)
            pr_warn("hdk_sc: enable типа %d не удался\n", c->ssp_type);
        else if (fn_set_delay)
            fn_set_delay(ssp_data, c->ssp_type, c->delay_ms, 0);
    }
    msleep(30);

    mutex_unlock(&hub_lock);
    atomic_inc(&reenable_count);
    if (ring)
        ring->reenable_count = (uint64_t)atomic_read(&reenable_count);
    pr_info("hdk_sc: переинициализация выполнена (reset=%d)\n", did_reset);
}

/* ======================== watchdog =============================== */
static void wd_fn(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    static unsigned long silent_since;
    uint64_t any = 0;
    int i;

    for (i = 0; i < 3; i++)
        any += __atomic_load_n(&ring->frames[i], __ATOMIC_RELAXED);

    if (!any) {
        if (!silent_since)
            silent_since = jiffies;
        else if (time_after(jiffies, silent_since + 5 * HZ)) {
            pr_warn("hdk_sc: нет кадров 5 с — восстановление (n=%d)\n",
                    (int)atomic_read(&reenable_count));
            atomic_set(&need_reenroll, 1);   /* переделают потоки-читатели */
            silent_since = jiffies;
        }
    } else {
        silent_since = 0;
    }
    if (!atomic_read(&stop))
        schedule_delayed_work(dwork, 2 * HZ);
}
static DECLARE_DELAYED_WORK(wd_work, wd_fn);

/* ===================== частоты в кольце ========================== */
static void rate_timer_fn(unsigned long unused);
static struct timer_list rate_timer;

static void rate_timer_fn(unsigned long unused)
{
    static uint64_t last[3];
    static unsigned long lastj;
    unsigned long now = jiffies;
    int i;

    if (!lastj)
        lastj = now;
    if (now - lastj < HZ)
        goto out;
    for (i = 0; i < 3; i++) {
        uint64_t f = __atomic_load_n(&ring->frames[i], __ATOMIC_RELAXED);
        double hz = (double)(f - last[i]) * HZ / (double)(now - lastj);
        ring->rate_mHz[i] = (int64_t)(hz * 1000.0);
        last[i] = f;
    }
    lastj = now;
    ring->open_fail = (uint64_t)atomic_read(&open_fail);
out:
    if (!atomic_read(&stop))
        mod_timer(&rate_timer, jiffies + HZ);
}

/* ===================== mmap: безопасно на vzalloc ================= */
static int hdk_mmap(struct file *file, struct vm_area_struct *vma)
{
    unsigned long size = vma->vm_end - vma->vm_start;

    if (!ring || size == 0 || size > ring_bytes)
        return -EINVAL;
    /* только постраничное отображение со смещения 0: память vzalloc
     * физически размечена по страницам, remap_vmalloc_range это требует. */
    if (vma->vm_pgoff || (size & ~PAGE_MASK))
        return -EINVAL;

    vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
    vma->vm_page_prot = PAGE_SHARED;
    /* remap_vmalloc_range принимает void* в этой версии ядра */
    return remap_vmalloc_range(vma, ring, size);
}

static long hdk_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    case HDK_IOCTL_RESET_CHUB:
        pr_warn("hdk_sc: sensorhub_reset по ioctl\n");
        if (driver_ok && ssp_data) {
            mutex_lock(&hub_lock);
            fn_hub_reset(ssp_data);
            total_resets++;
            mutex_unlock(&hub_lock);
        }
        return 0;
    case HDK_IOCTL_FORCE_REENROLL:
        atomic_set(&need_reenroll, 1);
        return 0;
    case HDK_IOCTL_GET_STATS: {
        struct hdk_stats st;
        int i;
        memset(&st, 0, sizeof(st));
        st.prod     = __atomic_load_n(&ring->prod, __ATOMIC_RELAXED);
        st.cons     = __atomic_load_n(&ring->cons, __ATOMIC_RELAXED);
        st.dropped  = __atomic_load_n(&ring->dropped, __ATOMIC_RELAXED);
        st.reenable_count = (uint64_t)atomic_read(&reenable_count);
        st.open_fail = (uint64_t)atomic_read(&open_fail);
        for (i = 0; i < HDK_FRAME_N; i++) {
            st.rate_mHz[i] = ring->rate_mHz[i];
            st.frames[i] = ring->frames[i];
        }
        st.driver_ok = driver_ok ? 1 : 0;
        if (driver_ok && ssp_data)
            st.hub_working = fn_is_working(ssp_data) ? 1 : 0;
        for (i = 0; i < 3; i++)
            st.ch_open[i] = CH_CFG[i].opened ? 1 : 0;
        if (copy_to_user((void __user *)arg, &st, sizeof(st)))
            return -EFAULT;
        return 0;
    }
    default:
        return -ENOTTY;
    }
}

static const struct file_operations hdk_fops = {
    .owner       = THIS_MODULE,
    .mmap        = hdk_mmap,
    .unlocked_ioctl = hdk_ioctl,
};

static struct miscdevice hdk_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = HDK_DEV_NAME,
    .fops  = &hdk_fops,
};

/* ========================== init/exit ============================= */
static int __init hdk_sc_init(void)
{
    int i, rc;

    pr_info("hdk_sc: init\n");

    ring_bytes = PAGE_ALIGN(sizeof(struct hdk_ring) +
                             HDK_RING_CAP * sizeof(struct hdk_frame));
    ring = vzalloc(ring_bytes);
    if (!ring)
        return -ENOMEM;
    ring->magic      = HDK_MAGIC;
    ring->version    = HDK_VERSION;
    ring->frame_size = (uint32_t) sizeof(struct hdk_frame);
    ring->ring_cap   = HDK_RING_CAP;
    ring_frames = (struct hdk_frame *)((uint8_t *)ring + sizeof(struct hdk_ring));

    mutex_init(&hub_lock);

    if (resolve_driver() == 0) {
        ssp_data = find_ssp_data();
        driver_ok = ssp_data != NULL;
        pr_info("hdk_sc: драйвер SSP %s, ssp_data=%px\n",
                driver_ok ? "доступен" : "НЕ найден", ssp_data);
    } else {
        pr_err("hdk_sc: символы драйвера недоступны\n");
    }

    rc = misc_register(&hdk_misc);
    if (rc) {
        pr_err("hdk_sc: misc_register: %d\n", rc);
        vfree(ring);
        ring = NULL;
        return rc;
    }

    /* Первичное включение датчиков и задание частот. */
    reenroll_channels();

    for (i = 0; i < 3; i++) {
        /*
         * kthread_create_on_cpu() создаёт поток сразу на нужном ядре —
         * это штатный способ привязки без cpumask_of()/set_bit()/sched_setaffinity
         * (их сигнатуры в этом ядре отличаются от документированных).
         */
        thr[i] = kthread_create_on_cpu(chan_thread, (void *)(long)i,
                                       i % 4, "hdk_sc%d", i);
        if (IS_ERR(thr[i])) {
            thr[i] = NULL;
            pr_err("hdk_sc: поток %d не создан: %ld\n", i, PTR_ERR(thr[i]));
            continue;
        }
#if defined(CONFIG_SCHED_FIFO)
        {
            struct sched_param sp = { .sched_priority = 98 };
            sched_setscheduler_nocheck(thr[i], SCHED_FIFO, &sp);
        }
#endif
        wake_up_process(thr[i]);
    }

    schedule_delayed_work(&wd_work, 2 * HZ);
    setup_timer(&rate_timer, rate_timer_fn, 0);
    mod_timer(&rate_timer, jiffies + HZ);

    pr_info("hdk_sc: готов. /dev/%s, кольцо %zu Б, потоков %d, "
            "водитель=%s\n", HDK_DEV_NAME, ring_bytes,
            thr[0] && thr[1] && thr[2] ? 3 : 0,
            driver_ok ? "да" : "нет");
    return 0;
}

static void __exit hdk_sc_exit(void)
{
    int i;

    atomic_set(&stop, 1);
    cancel_delayed_work_sync(&wd_work);
    del_timer_sync(&rate_timer);

    /* Потоки не спят в read (O_NONBLOCK), поэтому kthread_stop не зависнет. */
    for (i = 0; i < 3; i++)
        if (thr[i])
            kthread_stop(thr[i]);

    if (driver_ok && ssp_data)
        for (i = 0; i < 3; i++)
            enable_type(CH_CFG[i].ssp_type, 0);

    misc_deregister(&hdk_misc);
    vfree(ring);
    ring = NULL;
    pr_info("hdk_sc: выгружен (resets=%lu)\n", total_resets);
}

module_init(hdk_sc_init);
module_exit(hdk_sc_exit);