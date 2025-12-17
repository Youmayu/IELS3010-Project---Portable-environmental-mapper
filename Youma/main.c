#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/sensor.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------
 * Identification
 * --------------------------------------------------------------------- */
#define DEVICE_NAME   "IELS3010_NODEA_T3"
#define TEAM_ID       3
#define COMPANY_ID    0xBEEF

/* ---------------------------------------------------------------------
 * Advertising timing
 * --------------------------------------------------------------------- */
#define ADV_PERIOD_MS   2000
#define ADV_UNITS(ms)   ((ms) * 1000 / 625)

/* ---------------------------------------------------------------------
 * Payload layout (7 bytes total)
 *
 * [0..1] Company ID        (uint16, LE)
 * [2]    Team ID           (uint8)
 * [3..4] Sequence number   (uint16, LE)
 * [5..6] Die temperature   (int16, °C × 100)
 * --------------------------------------------------------------------- */
#define PAYLOAD_LEN 7

static uint8_t payload[PAYLOAD_LEN];
static const struct device *temp_sensor;

/* ---------------------------------------------------------------------
 * Build BLE payload
 * --------------------------------------------------------------------- */
static void build_payload(uint16_t seq, int16_t temp_x100)
{
    sys_put_le16(COMPANY_ID, &payload[0]);
    payload[2] = TEAM_ID;
    sys_put_le16(seq, &payload[3]);
    sys_put_le16((uint16_t)temp_x100, &payload[5]);
}

/* ---------------------------------------------------------------------
 * Read internal die temperature (°C × 100)
 * --------------------------------------------------------------------- */
static int16_t read_die_temperature(void)
{
    struct sensor_value value;

    if (sensor_sample_fetch(temp_sensor) < 0) {
        return 0;
    }

    if (sensor_channel_get(temp_sensor,
                           SENSOR_CHAN_DIE_TEMP,
                           &value) < 0) {
        return 0;
    }

    return (int16_t)(value.val1 * 100 + value.val2 / 10000);
}

/* ---------------------------------------------------------------------
 * Main
 * --------------------------------------------------------------------- */
void main(void)
{
    int err;
    uint16_t seq = 0;

    /* Enable Bluetooth */
    err = bt_enable(NULL);
    if (err) {
        printk("BLE init failed (%d)\n", err);
        return;
    }

    /* Get internal temperature sensor */
    temp_sensor = DEVICE_DT_GET_ANY(nordic_nrf_temp);
    if (!device_is_ready(temp_sensor)) {
        printk("Temperature sensor not ready\n");
        return;
    }

    /* Advertising data */
    static const char device_name[] = DEVICE_NAME;

    struct bt_data adv_data[] = {
        BT_DATA(BT_DATA_NAME_COMPLETE,
                device_name,
                sizeof(device_name) - 1),

        BT_DATA(BT_DATA_MANUFACTURER_DATA,
                payload,
                sizeof(payload)),
    };

    /* Advertising parameters (2 s interval) */
    const struct bt_le_adv_param adv_params = {
        .options      = BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = ADV_UNITS(ADV_PERIOD_MS),
        .interval_max = ADV_UNITS(ADV_PERIOD_MS),
    };

    /* Initial sample before advertising */
    int16_t temperature = read_die_temperature();
    build_payload(seq, temperature);

    err = bt_le_adv_start(&adv_params,
                          adv_data,
                          ARRAY_SIZE(adv_data),
                          NULL,
                          0);
    if (err) {
        printk("Advertising start failed (%d)\n", err);
        return;
    }

    printk("Node A advertising die temperature\n");

    while (1) {
        printk("seq=%u  temp=%d.%02d C\n",
               seq,
               temperature / 100,
               abs(temperature % 100));

        k_sleep(K_MSEC(ADV_PERIOD_MS));

        seq++;
        temperature = read_die_temperature();
        build_payload(seq, temperature);
        bt_le_adv_update_data(adv_data, ARRAY_SIZE(adv_data), NULL, 0);
    }
}
