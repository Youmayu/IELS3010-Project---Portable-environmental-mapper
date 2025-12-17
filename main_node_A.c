#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(node_a, LOG_LEVEL_INF);

/* Manufacturer Company ID:
 * 0xFFFF is fine for lab/prototype (not for commercial products).
 */
#define COMPANY_ID_TEST 0xFFFF

/* Packed payload (little-endian) */
struct payload_v1 {
	uint16_t seq;        /* increments */
	int16_t  t_c_x100;   /* temperature * 100 */
	uint16_t rh_x100;    /* humidity * 100 (dummy) */
	int32_t  lat_e7;     /* latitude * 1e7 (dummy) */
	int32_t  lon_e7;     /* longitude * 1e7 (dummy) */
	uint8_t  flags;      /* bit0 = LED state */
} __packed;

/* LED0 on DK */
static const struct gpio_dt_spec led =
	GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

/* Temperature device (internal die temperature) */
static const struct device *temp_dev;

/* Company ID (2 bytes) + payload */
static uint8_t mfg_buf[2 + sizeof(struct payload_v1)];

/* Read internal SoC temperature (die temp). Fallback to dummy if unavailable. */
static int16_t read_temp_x100(void)
{
	if (!temp_dev) {
		return 2300; /* 23.00 C dummy */
	}

	struct sensor_value val;

	/* Try to fetch and read die temperature */
	if (sensor_sample_fetch(temp_dev) == 0 &&
	    sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &val) == 0) {
		/* sensor_value: val1 = integer, val2 = micro (1e-6) */
		int32_t x100 = (int32_t)val.val1 * 100 + (int32_t)(val.val2 / 10000);
		return (int16_t)x100;
	}

	return 2300; /* fallback */
}

static void fill_payload(struct payload_v1 *p, uint16_t seq, bool led_on)
{
	/* Real temperature if possible */
	int16_t t_x100 = read_temp_x100();

	/* Dummy humidity (no RH sensor on nRF54L15DK) */
	uint16_t rh_x100 = 4500; /* 45.00% */

	/* Dummy GPS (no GNSS on nRF54L15DK) */
	int32_t lat_e7 = (int32_t)(59.9139 * 1e7); /* Oslo-ish */
	int32_t lon_e7 = (int32_t)(10.7522 * 1e7);

	p->seq      = sys_cpu_to_le16(seq);
	p->t_c_x100 = sys_cpu_to_le16(t_x100);
	p->rh_x100  = sys_cpu_to_le16(rh_x100);
	p->lat_e7   = sys_cpu_to_le32(lat_e7);
	p->lon_e7   = sys_cpu_to_le32(lon_e7);
	p->flags    = led_on ? 0x01 : 0x00;
}

int main(void)
{
	int err;

	/* LED init (optional) */
	if (led.port) {
		(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	}

	/* Enable Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return 0;
	}
	LOG_INF("Bluetooth enabled");

	/* Try to get internal temperature device.
	 * If not present in this SDK/board config, we keep temp_dev = NULL (dummy temp).
	 */
#if DT_HAS_COMPAT_STATUS_OKAY(nordic_nrf_temp)
	temp_dev = DEVICE_DT_GET_ONE(nordic_nrf_temp);
	if (!device_is_ready(temp_dev)) {
		LOG_WRN("Temp sensor not ready -> using dummy temp");
		temp_dev = NULL;
	} else {
		LOG_INF("Temp sensor ready (die temp)");
	}
#else
	LOG_WRN("No nordic_nrf_temp devicetree node -> using dummy temp");
	temp_dev = NULL;
#endif

	/* Prepare manufacturer buffer: company ID + payload */
	sys_put_le16(COMPANY_ID_TEST, &mfg_buf[0]);

	uint16_t seq = 0;
	bool led_on = false;

	struct payload_v1 payload;
	fill_payload(&payload, seq, led_on);
	memcpy(&mfg_buf[2], &payload, sizeof(payload));

	/* Advertising parameters (legacy, non-connectable is fine for demo) */
	struct bt_le_adv_param adv_param = {
		.options = BT_LE_ADV_OPT_USE_IDENTITY,
		.interval_min = BT_GAP_ADV_SLOW_INT_MIN,
		.interval_max = BT_GAP_ADV_SLOW_INT_MAX,
	};

	/* Advertising data */
const struct bt_data ad[] = {
    BT_DATA(BT_DATA_FLAGS, (uint8_t[]){ BT_LE_AD_NO_BREDR }, 1),

    /* Device name: "NODE_A" */
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),

    /* Manufacturer payload (Company ID + payload_v1) */
    BT_DATA(BT_DATA_MANUFACTURER_DATA,
            mfg_buf,
            sizeof(mfg_buf)),
};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("bt_le_adv_start failed (%d)", err);
		return 0;
	}
	LOG_INF("Advertising started (name=%s)", CONFIG_BT_DEVICE_NAME);

	while (1) {
		k_sleep(K_SECONDS(2));

		seq++;
		led_on = !led_on;

		if (led.port) {
			gpio_pin_set_dt(&led, (int)led_on);
		}

		fill_payload(&payload, seq, led_on);
		memcpy(&mfg_buf[2], &payload, sizeof(payload));

		/* Update advertising data (manufacturer payload changed) */
		err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			LOG_WRN("bt_le_adv_update_data failed (%d)", err);
		}

		/* Log what we are sending */
		int16_t t_x100 = (int16_t)sys_le16_to_cpu(payload.t_c_x100);
		uint16_t rh_x100 = sys_le16_to_cpu(payload.rh_x100);

		LOG_INF("tx seq=%u led=%d t=%.2fC rh=%.2f%% (rh/gps are dummy)",
		        seq, led_on,
		        ((double)t_x100) / 100.0,
		        ((double)rh_x100) / 100.0);
	}
}
