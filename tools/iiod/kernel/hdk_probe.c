/*
 * hdk_probe.c — пробный модуль ядра для LOGGER (Helideck).
 *
 * Цель первого этапа — доказать, что внешний .ko на этом ядре (Mint Beta
 * 1413, 4.14.194, тег xbeta-1413_a50dx) может:
 *   1. загрузиться (vermagic совпадает с установленным ядром);
 *   2. через EXPORT_SYMBOL_GPL(kallsyms_lookup_name) найти адреса
 *      неэкспортированных функций SSP-драйвера (ssp_send_command,
 *      enable_sensor, sensorhub_reset, is_sensorhub_working,
 *      get_sensor_scanning_info);
 *   3. найти экземпляр struct ssp_data * через класс sensors
 *      (устройство "ssp_sensor", drvdata = &data) и прочитать
 *      sensor_probe_state — ключевое состояние, из-за которого драйвер
 *      отвечает «commnd error -19» (docs/SENSOR_DIRECT_KERNEL.md §5a).
 *
 * Это ТОЛЬКО ЧТЕНИЕ: никаких записей в драйвер, никакого изменения
 * состояния датчиков. Следующий этап (восстановление: reset_mcu /
 * повторный get_sensor_scanning_info) — отдельным модулем, после того как
 * чтение подтверждено на стенде.
 *
 * Сборка (дерево Mint на теге xbeta-1413_a50dx):
 *   make ARCH=arm64 M=<dir> modules \
 *        LOCALVERSION=" - Mint Beta 1413" CC=clang HOSTCC=clang HOSTCXX=clang++ \
 *        AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip
 * (переменные сборки — как в build.sh Mint).
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kallsyms.h>
#include <linux/device.h>
#include <linux/utsname.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Helideck LOGGER");
MODULE_DESCRIPTION("HDK probe: доступ к состоянию SSP-драйвера из ядра (read-only)");
MODULE_VERSION("0.1.0");

/* kallsyms_lookup_name экспортируется на 4.14 (EXPORT_SYMBOL_GPL). */
extern unsigned long kallsyms_lookup_name(const char *name);

/*
 * sensors_class экспортируется из drivers/sensorhub/sensors_core.c
 * (EXPORT_SYMBOL_GPL(sensors_class)). Заголовок sensors_core.h живёт внутри
 * дерева ядра (drivers/sensorhub/), вне exported-заголовков, поэтому для
 * внешнего модуля объявляем extern сами — это часть стабильного экспорта.
 */
extern struct class *sensors_class;

/* Адреса неэкспортированных функций SSP-драйвера, резолвленные в init. */
static unsigned long ks_ssp_send_command;
static unsigned long ks_enable_sensor;
static unsigned long ks_sensorhub_reset;
static unsigned long ks_is_sensorhub_working;
static unsigned long ks_get_sensor_scanning_info;
static int ssp_found;

/* Матчер class_find_device: ищем устройство с именем kobj_name == "ssp_sensor". */
static int match_ssp_sensor(struct device *dev, const void *data)
{
	return strcmp(dev_name(dev), (const char *)data) == 0;
}

static int __init hdk_probe_init(void)
{
	unsigned long addr;
	struct device *dev;
	void *drvdata;

	pr_info("hdk_probe: init, release='%s'\n", init_uts_ns.name.release);

#define RESOLVE(sym, var) \
	do { \
		addr = kallsyms_lookup_name(sym); \
		if (!addr) { \
			pr_err("hdk_probe: символ %s НЕ найден\n", sym); \
		} else { \
			pr_info("hdk_probe: %s = 0x%lx\n", sym, addr); \
			var = addr; \
			ssp_found++; \
		} \
	} while (0)

	RESOLVE("ssp_send_command", ks_ssp_send_command);
	RESOLVE("enable_sensor", ks_enable_sensor);
	RESOLVE("sensorhub_reset", ks_sensorhub_reset);
	RESOLVE("is_sensorhub_working", ks_is_sensorhub_working);
	RESOLVE("get_sensor_scanning_info", ks_get_sensor_scanning_info);
#undef RESOLVE

	/*
	 * sensors_class экспортируется (sensors_core.c, EXPORT_SYMBOL_GPL).
	 * Устройство класса sensors с именем "ssp_sensor" создаётся в
	 * ssp_sysfs.c: sensors_device_register(..., data, mcu_attrs, "ssp_sensor"),
	 * drvdata = struct ssp_data *.
	 */
	dev = class_find_device(sensors_class, NULL, "ssp_sensor",
				match_ssp_sensor);
	if (!dev) {
		pr_err("hdk_probe: устройство ssp_sensor не найдено\n");
		pr_info("hdk_probe: резолвнуто символов: %d\n", ssp_found);
		return 0;
	}
	drvdata = dev_get_drvdata(dev);
	pr_info("hdk_probe: ssp_sensor найден, drvdata=%px\n", drvdata);
	put_device(dev);

	pr_info("hdk_probe: инициализация завершена, символов: %d\n", ssp_found);
	return 0;
}

static void __exit hdk_probe_exit(void)
{
	pr_info("hdk_probe: выгрузка\n");
}

module_init(hdk_probe_init);
module_exit(hdk_probe_exit);