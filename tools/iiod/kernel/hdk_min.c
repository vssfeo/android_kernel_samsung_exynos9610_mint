/*
 * hdk_min.c — минимальный диагностический модуль.
 *
 * Цель: БИСЕКЦИЯ отказа insmod ("Exec format error" при полном молчании ядра).
 * Этот модуль НЕ импортирует ни одного внешнего символа: ни kallsyms_lookup_name,
 * ни sensors_class, ни функций ядра. Если он тоже не загрузится — причина
 * в СБОРКЕ (конфиг ядра / modversions / vermagic), а не в символах.
 * Если загрузится — отказ привязан к импортируемым символам и их CRC.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Helideck LOGGER");
MODULE_DESCRIPTION("HDK min probe: без внешних символов (бисекция insmod)");
MODULE_VERSION("0.1.0");

static int hdk_min_init(void)
{
    pr_info("hdk_min: загружен, ядро '%s'\n", init_uts_ns.name.release);
    return 0;
}

static void hdk_min_exit(void)
{
    pr_info("hdk_min: выгружен\n");
}

module_init(hdk_min_init);
module_exit(hdk_min_exit);
