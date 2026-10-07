/*
 * hdk_min.c — минимальный диагностический модуль.
 *
 * Цель: БИСЕКЦИЯ отказа insmod ("Exec format error" при полном молчании ядра).
 * Этот модуль НЕ импортирует ни одного внешнего символа: ни kallsyms_lookup_name,
 * ни sensors_class, ни init_uts_ns, ни функций ядра. Если он тоже не загрузится —
 * причина в СБОРКЕ (конфиг ядра / modversions / vermagic), а не в символах.
 * Если загрузится — отказ привязан к импортируемым символам и их CRC.
 *
 * init_uts_ns намеренно НЕ используется: на этом ядре он не экспортирован,
 * а при CONFIG_MODVERSIONS импорт символа без экспорта даёт отказ загрузки.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Helideck LOGGER");
MODULE_DESCRIPTION("HDK min probe: no external symbols (insmod bisect)");
MODULE_VERSION("0.1.0");

static int hdk_min_init(void)
{
    pr_info("hdk_min: loaded, running kernel\n");
    return 0;
}

static void hdk_min_exit(void)
{
    pr_info("hdk_min: unloaded\n");
}

module_init(hdk_min_init);
module_exit(hdk_min_exit);
