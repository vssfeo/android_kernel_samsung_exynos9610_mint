/*
 * hdk_sensor_core.ko — боевой модуль прямого доступа к датчикам (LOGGER/Helideck).
 *
 * ЧТО ЭТО
 *   Замена связки «демон hdk_iiod + подмена hals.conf» одним модулем ядра.
 *   Модуль становится ЕДИНСТВЕННЫМ владельцем IIO-буферов сенсоров
 *   (/dev/iio:device*, включены через SSP-драйвер), читает их реальным
 *   realtime-потоком (SCHED_FIFO, привязка к ядру) и отдаёт кадры
 *   приложению через mmap-кольцо без единого syscall на кадр.
 *
 *   Нативный HAL датчиков (sensors.sensorhub.so) при этом гарантированно
 *   отключён: он физически не может открыть /dev/iio, который держит модуль
 *   (второму владельцу IIO отвечает Device or resource busy — проверено в
 *   STATE.md §3a / docs/SENSOR_DIRECT_KERNEL.md §5). Не нужно ни подменять
 *   hals.conf, ни убивать sensorservice — буфер недоступен, и всё.
 *
 * САМОВОССТАНОВЛЕНИЕ (главная ценность)
 *   Watchdog по кадрам. Если каналы молчат:
 *     1) get_sensor_scanning_info() — перечитать sensor_probe_state у хаба
 *        (это то состояние, из-за которого enable_sensor падает с -19);
 *     2) disable+enable каждого типа (снятие и включение, как в hdk_iiod);
 *     3) если хаб не отвечает — sensorhub_reset() из ядра — то, что sysfs
 *        mcu_reset запрещает userspace (STATE.md §3a);
 *     4) переоткрыть /dev/iio и продолжить.
 *   Это лечит состояние «No Sensors on the device» БЕЗ перезагрузки телефона.
 *
 * ПРИОРИТЕТ
 *   kthread'ы создаются с SCHED_FIFO 98 и привязываются к отдельным ядрам —
 *   выше, чем любой userspace-процесс, включая system_server и HAL.
 *
 * АРХИТЕКТУРА
 *   CHUB → SSP-драйвер → /dev/iio:deviceN (владеет модуль)
 *        → kthread считывает кадры → кольцо в ядре → /dev/hdk_sensors (mmap)
 *        → приложение (LOGGER) читает mmap без syscalls.
 *
 * СБОРКА (в дереве Mint xbeta-1413_a50dx, см. .github/workflows из ветки
 * hdk/probe-1413): kallsyms_lookup_name, sensors_class, set_delay_legacy_sensor
 * резолвятся по имени; модуль линкуется только с kallsyms_lookup_name.
 *
 * ФОРМАТ КАДРА (совместим с sensor_direct.c, 24 байта):
 *   int64 t_ns; uint16 ch; uint16 n; int16 v[6];
 *   ch: 0=acc, 1=gyro, 2=mag, 3=gyro+bias, 4=mag+bias (как в hdk_iiod).
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kallsyms.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/ktime.h>
#include <linux/uaccess.h>
#include <linux/freezer.h>
#include <linux/spinlock.h>
#include <linux/utsname.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Helideck LOGGER");
MODULE_DESCRIPTION("hdk_sensor_core: прямой realtime-доступ к датчикам из ядра");
MODULE_VERSION("1.0.0");

/* ============================== ABI =============================== */
#define HDK_MAGIC      0x48444B52u          /* "HDKR" */
#define HDK_VERSION    1u

#define HDK_RING_CAP   8192                 /* кадров в кольце (степень 2) */
#define HDK_FRAME_N    5                    /* каналов в ABI (как hdk_iiod) */
#define HDK_DEV_NAME   "hdk_sensors"

/* Кадр: совместим с форматом hdk_iiod / sensor_direct.c (24 Б). */
struct hdk_frame {
    int64_t  t_ns;
    uint16_t ch;
    uint16_t n;
    int16_t  v[6];
} __packed;

/* Шапка кольца. Читается приложением через mmap. */
struct hdk_ring {
    uint32_t magic;
    uint32_t version;
    uint32_t frame_size;      /* sizeof(struct hdk_frame) */
    uint32_t ring_cap;
    volatile uint64_t prod;   /* пишет модуль */
    volatile uint64_t cons;   /* читает приложение */
    volatile uint64_t dropped;
    volatile int64_t  rate_mHz[HDK_FRAME_N];
    volatile uint64_t frames[HDK_FRAME_N];
};

/* ioctl: диагностика; кольцо доступно через mmap без syscalls на кадр. */
#define HDK_IOC_MAGIC 'H'
#define HDK_IOCTL_RESET_CHUB   _IO(HDK_IOC_MAGIC, 1)  /* принудительный reset хаба */
#define HDK_IOCTL_GET_STATS    _IOR(HDK_IOC_MAGIC, 2, struct hdk_stats)
struct hdk_stats {
    uint64_t prod, cons, dropped;
    int64_t  rate_mHz[HDK_FRAME_N];
    uint64_t frames[HDK_FRAME_N];
    uint32_t hub_working;
    uint32_t reset_count;
    uint32_t ch_open[3];          /* 1 если канал открыт */
    char     release[64];
};

/* ===================== символы SSP-драйвера ====================== */
struct ssp_data;

extern unsigned long kallsyms_lookup_name(const char *name);
extern struct class *sensors_class;

static int (*fn_enable_sensor)(struct ssp_data *, unsigned int, u8 *, int);
static int (*fn_disable_sensor)(struct ssp_data *, unsigned int, u8 *, int);
static int (*fn_set_delay_legacy)(struct ssp_data *, unsigned int, int, int);
static int (*fn_get_scanning)(struct ssp_data *);
static int (*fn_sensorhub_reset)(void *);
static bool (*fn_is_working)(void *);
static int has_driver_access;

/* ======================= каналы (ABI hdk_iiod) =================== */
enum { CH_ACC = 0, CH_GYRO = 1, CH_MAG = 2, CH_GYRO_B = 3, CH_MAG_B = 4 };

struct hdk_chan {
    const char *name;        /* /dev/iio:deviceN */
    int  ssp_type;           /* тип SSP: 1 acc, 16 uncal_gyro, 14 uncal_mag */
    int  ring_ch;            /* канал в кольце */
    int  delay_ms;           /* период опроса */
    /* runtime */
    struct file *filp;
    uint8_t *buf;
    int  frame_bytes;
} CH_CFG[3] = {
    { "/dev/iio:device1", 1,  CH_ACC,   4, NULL, NULL, 0 },
    { "/dev/iio:device7", 16, CH_GYRO_B, 4, NULL, NULL, 0 },
    { "/dev/iio:device6", 14, CH_MAG_B,  8, NULL, NULL, 0 },
};

/* ============================= состояние ========================= */
static struct hdk_ring *ring;
static struct hdk_frame *ring_frames;
static struct ssp_data *ssp_data;
static struct class *ssp_dev_class;

static struct task_struct *thr[3];
static wait_queue_head_t wq[3];
static atomic_t stop = ATOMIC_INIT(0);
static struct mutex enroll_lock;   /* реинициализация каналов */
static uint32_t reset_count;

/* ====================== функции драйвера ========================= */
static int resolve_driver_symbols(void)
{
    unsigned long a;

#define RES(name, ptr, sig) \
    do { \
        a = kallsyms_lookup_name(name); \
        if (!a) { pr_err("hdk_sc: символ %s НЕ найден\n", name); return -ENOENT; } \
        *(unsigned long *)&(ptr) = a; \
    } while (0)

    /* set_delay_legacy_sensor: (data, type, sampling_period_ms, max_latency_ms) */
    RES("enable_sensor", fn_enable_sensor, sig);
    RES("disable_sensor", fn_disable_sensor, sig);
    RES("set_delay_legacy_sensor", fn_set_delay_legacy, sig);
    RES("get_sensor_scanning_info", fn_get_scanning, sig);
    RES("sensorhub_reset", fn_sensorhub_reset, sig);
    RES("is_sensorhub_working", fn_is_working, sig);
#undef RES
    return 0;
}

/* Найти struct ssp_data * — drvdata устройства "ssp_sensor" класса sensors. */
static int match_ssp_sensor(struct device *dev, const void *data)
{
    return strcmp(dev_name(dev), (const char *)data) == 0;
}

static struct ssp_data *find_ssp_data(void)
{
    struct device *dev;
    struct ssp_data *d;

    dev = class_find_device(sensors_class, NULL, "ssp_sensor", match_ssp_sensor);
    if (!dev) {
        pr_err("hdk_sc: устройство ssp_sensor не найдено\n");
        return NULL;
    }
    d = dev_get_drvdata(dev);
    put_device(dev);
    return d;
}

/* Включить/выключить тип через API драйвера (то же, что sysfs enable). */
static int hdk_enable_type(int ssp_type, int on)
{
    u8 buf[2] = {0, 0};
    if (!has_driver_access || !ssp_data)
        return -ENODEV;
    if (on)
        return fn_enable_sensor(ssp_data, ssp_type, buf, sizeof(buf));
    return fn_disable_sensor(ssp_data, ssp_type, buf, sizeof(buf));
}

/* Размер кадра IIO — читаем из sysfs драйвера (как в hdk_iiod). */
static int chan_frame_bytes(const char *dev)
{
    const char *base = strrchr(dev, '/');
    char path[96];
    char line[64] = {0};
    struct file *f;
    int n, bits = 0;
    loff_t pos = 0;

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
    /* Формат "le:s112/112>>0" -> bits = 112; кадр = bits/8 байт. */
    {
        char *s = strrchr(line, '/');
        if (!s)
            return 0;
        bits = simple_strtol(s + 1, NULL, 10);
    }
    return bits > 0 ? bits / 8 : 0;
}

static int chan_open(int idx)
{
    struct hdk_chan *c = &CH_CFG[idx];

    c->frame_bytes = chan_frame_bytes(c->name);
    if (c->frame_bytes <= 0) {
        pr_warn("hdk_sc: %s: нет scan_elements, канал пропущен\n", c->name);
        return -EINVAL;
    }
    c->buf = kzalloc(16384, GFP_KERNEL);
    if (!c->buf)
        return -ENOMEM;
    c->filp = filp_open(c->name, O_RDONLY, 0);
    if (IS_ERR(c->filp)) {
        pr_warn("hdk_sc: %s: %ld (HAL держит буфер?)\n", c->name,
                PTR_ERR(c->filp));
        c->filp = NULL;
        kfree(c->buf);
        c->buf = NULL;
        return -EBUSY;
    }
    pr_info("hdk_sc: %s открыт, кадр %d Б, delay %d мс\n",
            c->name, c->frame_bytes, c->delay_ms);
    return 0;
}

static void chan_close_all(void)
{
    int i;
    for (i = 0; i < 3; i++) {
        struct hdk_chan *c = &CH_CFG[i];
        if (c->filp) {
            filp_close(c->filp, NULL);
            c->filp = NULL;
        }
        kfree(c->buf);
        c->buf = NULL;
    }
}

/* Полная переустановка каналов: disable→enable→ODR→open. Возврат 0 если ОК. */
static int reenroll_channels(void)
{
    int i, rc = 0;

    mutex_lock(&enroll_lock);

    /* 1. Снять датчики. */
    for (i = 0; i < 3; i++)
        hdk_enable_type(CH_CFG[i].ssp_type, 0);
    msleep(50);

    /* 2. Если хаб не отвечает — сбросить его (sysfs mcu_reset запрещён). */
    if (has_driver_access && ssp_data && !fn_is_working(ssp_data)) {
        pr_warn("hdk_sc: хаб не отвечает — sensorhub_reset (из ядра)\n");
        fn_sensorhub_reset(ssp_data);
        reset_count++;
        msleep(200);
    }

    /* 3. Перечитать состояние датчиков (лечит sensor_probe_state == 0). */
    if (has_driver_access && ssp_data)
        fn_get_scanning(ssp_data);

    /* 4. Включить датчики и задать ODR. */
    for (i = 0; i < 3; i++) {
        struct hdk_chan *c = &CH_CFG[i];
        if (hdk_enable_type(c->ssp_type, 1) < 0) {
            pr_warn("hdk_sc: enable типа %d не удался\n", c->ssp_type);
            rc = -1;
        } else if (fn_set_delay_legacy &&
                   fn_set_delay_legacy(ssp_data, c->ssp_type, c->delay_ms, 0) < 0) {
            pr_warn("hdk_sc: set_delay типа %d не удался\n", c->ssp_type);
        }
    }
    msleep(30);

    /* 5. Переоткрыть каналы. */
    chan_close_all();
    for (i = 0; i < 3; i++)
        if (chan_open(i) < 0)
            rc = -1;

    mutex_unlock(&enroll_lock);
    return rc;
}

/* ====================== поток чтения канала ====================== */
static int chan_thread(void *arg)
{
    int idx = (int)(long)arg;
    struct hdk_chan *c = &CH_CFG[idx];

    allow_signal(SIGKILL);
    pr_info("hdk_sc: поток канала %d (%s) стартовал, pid %d\n",
            idx, c->name, task_pid_nr(current));

    while (!atomic_read(&stop)) {
        ssize_t r;
        struct file *f;

        /* Ждём открытия канала (может быть после реинролла). */
        while (!atomic_read(&stop) && !(f = READ_ONCE(c->filp)))
            msleep(20);
        if (atomic_read(&stop))
            break;
        if (!c->buf)
            break;

        r = kernel_read(f, c->buf, 16384, &(loff_t){0});
        if (r <= 0) {
            if (r < 0 && (r == -EINTR || r == -ERESTARTSYS))
                continue;
            /* канал упал — подождём, watchdog пересоздаст */
            msleep(10);
            continue;
        }

        {
            int frames = (int) r / c->frame_bytes;
            int need = c->frame_bytes;
            int i;
            for (i = 0; i < frames; i++) {
                const uint8_t *p = c->buf + (size_t) i * need;
                struct hdk_frame fr;
                int off, k;
                uint64_t prod, slot;
                uint32_t t = 0;

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
                /* timestamp в конце кадра (после значений) */
                off = need - 8;
                for (k = 7; k >= 0; k--)
                    t = (t << 8) | p[off + k];
                fr.t_ns = (int64_t) t;

                prod = __atomic_load_n(&ring->prod, __ATOMIC_RELAXED);
                slot = prod & (HDK_RING_CAP - 1);
                if (prod - __atomic_load_n(&ring->cons, __ATOMIC_RELAXED)
                        >= HDK_RING_CAP)
                    __atomic_add_fetch(&ring->dropped, 1, __ATOMIC_RELAXED);
                ring_frames[slot] = fr;
                __atomic_store_n(&ring->prod, prod + 1, __ATOMIC_RELEASE);
                __atomic_add_fetch(&ring->frames[idx], 1, __ATOMIC_RELAXED);
            }
        }
    }
    pr_info("hdk_sc: поток канала %d завершён\n", idx);
    return 0;
}

/* ======================== watchdog =============================== */
static struct delayed_work wd_work;

static void wd_fn(struct work_struct *work)
{
    static unsigned long last_silent;
    int i;
    uint64_t any = 0;

    for (i = 0; i < 3; i++)
        any += __atomic_load_n(&ring->frames[i], __ATOMIC_RELAXED);

    if (any == 0) {
        if (!last_silent)
            last_silent = jiffies;
        else if (time_after(jiffies, last_silent + 5 * HZ)) {
            pr_warn("hdk_sc: кадров нет 5 с — запускаю восстановление (" 
                    "pubcount=%u)\n", ++reset_count);
            reenroll_channels();
            last_silent = 0;
        }
    } else {
        last_silent = 0;
    }
    if (!atomic_read(&stop))
        schedule_delayed_work(&wd_work, 2 * HZ);
}

/* ==================== misc-устройство ============================ */
static int hdk_mmap(struct file *file, struct vm_area_struct *vma)
{
    unsigned long size = vma->vm_end - vma->vm_start;
    unsigned long pfn;
    int ret;

    if (size > (sizeof(struct hdk_ring) + HDK_RING_CAP * sizeof(struct hdk_frame)))
        return -EINVAL;

    /* Кольцо жило в kmalloc'd памяти; отдадим страницы в память приложению.
     * Проще всего: remap на зарезервированную память нет; используем
     * shmem-подобный трюк: страницы из alloc_pages. На практике для probe
     * и скорости используем remap_pfn_range на виртуальную память ядра,
     * доступную через физическую — здесь для простоты используется
     * vm_insert_page для каждой страницы кольца. */
    pfn = virt_to_phys((void *)ring) >> PAGE_SHIFT;
    ret = remap_pfn_range(vma, vma->vm_start, pfn,
                          size, vma->vm_page_prot);
    return ret;
}

static long hdk_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    case HDK_IOCTL_RESET_CHUB:
        pr_warn("hdk_sc: принудительный sensorhub_reset по ioctl\n");
        if (has_driver_access && ssp_data) {
            fn_sensorhub_reset(ssp_data);
            reset_count++;
        }
        return 0;
    case HDK_IOCTL_GET_STATS: {
        struct hdk_stats st;
        int i;
        memset(&st, 0, sizeof(st));
        st.prod = __atomic_load_n(&ring->prod, __ATOMIC_RELAXED);
        st.cons = __atomic_load_n(&ring->cons, __ATOMIC_RELAXED);
        st.dropped = __atomic_load_n(&ring->dropped, __ATOMIC_RELAXED);
        for (i = 0; i < HDK_FRAME_N; i++) {
            st.rate_mHz[i] = ring->rate_mHz[i];
            st.frames[i] = ring->frames[i];
        }
        st.reset_count = reset_count;
        if (has_driver_access && ssp_data)
            st.hub_working = fn_is_working(ssp_data);
        for (i = 0; i < 3; i++)
            st.ch_open[i] = CH_CFG[i].filp != NULL;
        strlcpy(st.release, init_uts_ns.name.release, sizeof(st.release));
        if (copy_to_user((void __user *)arg, &st, sizeof(st)))
            return -EFAULT;
        return 0;
    }
    default:
        return -ENOTTY;
    }
}

static const struct file_operations hdk_fops = {
    .owner = THIS_MODULE,
    .mmap = hdk_mmap,
    .unlocked_ioctl = hdk_ioctl,
};

static struct miscdevice hdk_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = HDK_DEV_NAME,
    .fops = &hdk_fops,
};

/* ========================== init/exit ============================ */
static void update_rates(void)
{
    static uint64_t last[3];
    static unsigned long lastj;
    unsigned long now = jiffies;
    int i;
    unsigned long dt;

    if (!lastj)
        lastj = now;
    dt = now - lastj;
    if (dt < HZ)
        return;
    for (i = 0; i < 3; i++) {
        uint64_t f = __atomic_load_n(&ring->frames[i], __ATOMIC_RELAXED);
        double hz = (double)(f - last[i]) * HZ / dt;
        ring->rate_mHz[i] = (int64_t)(hz * 1000.0);
        last[i] = f;
    }
    lastj = now;
}

static void rate_timer_fn(unsigned long unused)
{
    update_rates();
    mod_timer(&rate_timer, jiffies + HZ);
}
static struct timer_list rate_timer;

static int __init hdk_sc_init(void)
{
    int i, rc = 0;
    size_t bytes;

    pr_info("hdk_sc: init, kernel %s\n", init_uts_ns.name.release);

    /* 1. Кольцо. */
    bytes = sizeof(struct hdk_ring) + HDK_RING_CAP * sizeof(struct hdk_frame);
    if (bytes < PAGE_SIZE)
        bytes = PAGE_SIZE;
    ring = kzalloc(PAGE_ALIGN(bytes), GFP_KERNEL);
    if (!ring)
        return -ENOMEM;
    ring->magic = HDK_MAGIC;
    ring->version = HDK_VERSION;
    ring->frame_size = sizeof(struct hdk_frame);
    ring->ring_cap = HDK_RING_CAP;
    ring_frames = (struct hdk_frame *)((uint8_t *)ring + sizeof(struct hdk_ring));

    mutex_init(&enroll_lock);

    /* 2. Доступ к драйверу SSP. */
    if (resolve_driver_symbols() == 0) {
        ssp_data = find_ssp_data();
        has_driver_access = ssp_data != NULL;
        if (!has_driver_access)
            pr_err("hdk_sc: ssp_data не найден — работаем только как кольцо\n");
        else
            pr_info("hdk_sc: драйвер SSP доступен, ssp_data=%px\n", ssp_data);
    } else {
        pr_err("hdk_sc: символы драйвера недоступны — только кольцо\n");
    }

    /* 3. Регистрация устройства. */
    rc = misc_register(&hdk_misc);
    if (rc) {
        pr_err("hdk_sc: misc_register: %d\n", rc);
        kfree(ring);
        ring = NULL;
        return rc;
    }

    /* 4. Первичный заход в каналы. */
    reenroll_channels();

    /* 5. Потоки чтения. */
    init_waitqueue_head(&wq[0]);
    init_waitqueue_head(&wq[1]);
    init_waitqueue_head(&wq[2]);
    for (i = 0; i < 3; i++) {
        thr[i] = kthread_run(chan_thread, (void *)(long)i, "hdk_sc%d", i);
        if (IS_ERR(thr[i])) {
            pr_err("hdk_sc: поток %d не создан: %ld\n", i, PTR_ERR(thr[i]));
            thr[i] = NULL;
            continue;
        }
        /* realtime */
        {
            struct sched_param sp = { .sched_priority = 98 };
            sched_setscheduler_nocheck(thr[i], SCHED_FIFO, &sp);
            /* привязать к ядру i (из доступных) */
            set_cpus_allowed_ptr(thr[i], cpumask_of(i % num_online_cpus()));
        }
    }

    /* 6. Watchdog. */
    INIT_DELAYED_WORK(&wd_work, wd_fn);
    schedule_delayed_work(&wd_work, 2 * HZ);

    /* 7. Частоты в кольце. */
    setup_timer(&rate_timer, rate_timer_fn, 0);
    mod_timer(&rate_timer, jiffies + HZ);

    pr_info("hdk_sc: готов, /dev/%s, кольцо %zu Б, потоков %d\n",
            HDK_DEV_NAME, bytes, 3);
    return 0;
}

static void __exit hdk_sc_exit(void)
{
    int i;
    atomic_set(&stop, 1);
    cancel_delayed_work_sync(&wd_work);
    del_timer_sync(&rate_timer);
    for (i = 0; i < 3; i++)
        if (thr[i] && !IS_ERR(thr[i])) {
            wake_up_process(thr[i]);
            kthread_stop(thr[i]);
        }
    chan_close_all();
    if (has_driver_access)
        /* мягкая уборка: выключим типы */
        for (i = 0; i < 3; i++)
            hdk_enable_type(CH_CFG[i].ssp_type, 0);
    misc_deregister(&hdk_misc);
    kfree(ring);
    ring = NULL;
    pr_info("hdk_sc: выгружен\n");
}

module_init(hdk_sc_init);
module_exit(hdk_sc_exit);