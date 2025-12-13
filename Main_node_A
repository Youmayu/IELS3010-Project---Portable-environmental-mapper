#include <zephyr/kernel.h>                 // k_sleep(), k_uptime_get(), osv.
#include <zephyr/types.h>                  // faste heltallstyper (uint16_t osv.)
#include <zephyr/sys/byteorder.h>          // sys_cpu_to_le16/le32 for endianness
#include <zephyr/bluetooth/bluetooth.h>    // bt_enable(), advertising API
#include <zephyr/bluetooth/hci.h>          // HCI/BT_DATA typer og konstanter
#include <zephyr/drivers/gpio.h>           // GPIO driver (LED)
#include <zephyr/sys/util.h>               // CLAMP(), ARRAY_SIZE()
#include <zephyr/logging/log.h>            // LOG_INF/LOG_ERR

/* Logger-tag for dette programmet (vises i logg/serial) */
LOG_MODULE_REGISTER(node_a, LOG_LEVEL_INF);

/*
 * Manufacturer Company ID i advertising.
 * 0xFFFF er "test"/prototype ID (ikke en ekte tildelt company ID).
 */
#define COMPANY_ID_TEST 0xFFFF

/*
 * Payload-format vi sender i BLE advertising (manufacturer data).
 * __packed er viktig slik at kompilatoren ikke legger inn padding mellom feltene.
 *
 * Alle felt sendes i little-endian (BLE/Zephyr forventer ofte det).
 */
struct payload_v1 {
    uint16_t seq;        // sekvensnummer (teller opp for hver sending)
    int16_t  t_c_x100;   // temperatur i grader C * 100 (f.eks. 2345 = 23.45°C)
    uint16_t rh_x100;    // relativ fuktighet i % * 100 (f.eks. 4550 = 45.50%)
    int32_t  lat_e7;     // latitude * 1e7 (GPS-format ofte brukt i embedded)
    int32_t  lon_e7;     // longitude * 1e7
    uint8_t  flags;      // statusbiter; her bruker vi bit0 = LED-state
} __packed;

/*
 * Henter "led0" fra device tree alias (standard på DK).
 * Hvis boardet ikke har led0 alias, blir led.port = NULL (fallback).
 */
static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

/*
 * Buffer for manufacturer data:
 *  - 2 bytes company id (little-endian)
 *  - resten er payload_v1
 */
static uint8_t mfg_buf[2 + sizeof(struct payload_v1)];

/*
 * Fyller inn payload.
 * Merk: per nå bruker vi dummy-data for temp/RH/GPS.
 * (Bra for å demonstrere BLE datapassasje uten å være avhengig av sensor/GNSS.)
 */
static void fill_payload(struct payload_v1 *p, uint16_t seq, bool led_on)
{
    /* "Basisverdier" som vi varierer rundt (dummy) */
    int32_t base_temp = 2300;    // 23.00°C
    int32_t base_rh   = 4500;    // 45.00%

    /*
     * Litt variasjon slik at mottaker/visualisering ser at data endrer seg.
     * (Syntetisk signal: temp varierer +/- 5°C, RH varierer +/- 4.5%)
     */
    int32_t t  = base_temp + (int32_t)(50 * (seq % 20) - 500);
    int32_t rh = base_rh   + (int32_t)(30 * (seq % 30) - 450);

    /* Legg inn felt og konverter til little-endian før sending */
    p->seq      = sys_cpu_to_le16(seq);
    p->t_c_x100 = sys_cpu_to_le16((int16_t)t);
    p->rh_x100  = sys_cpu_to_le16((uint16_t)CLAMP(rh, 0, 10000));  // 0.00%–100.00%

    /*
     * Dummy posisjon (Oslo-ish).
     * Bytt disse variablene ut med GNSS-data, for ekte GPS verdier.
     */
    int32_t lat = (int32_t)(59.9139 * 1e7);
    int32_t lon = (int32_t)(10.7522 * 1e7);
    p->lat_e7   = sys_cpu_to_le32(lat);
    p->lon_e7   = sys_cpu_to_le32(lon);

    /* flags: bit0 forteller LED state */
    p->flags = led_on ? 0x01 : 0x00;
}

int main(void)
{
    int err;

    /* Konfigurer LED som output hvis den finnes på boardet */
    if (led.port) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    }

    /* Start Bluetooth stack */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed (%d)", err);
        return 0;
    }
    LOG_INF("Bluetooth enabled");

    /*
     * Advertising parametere:
     * - BT_LE_ADV_OPT_USE_IDENTITY: bruker identity address (mer "stabil" identitet)
     * - Intervall settes til "slow" (energivennlig, men kan økes for raskere oppdatering)
     */
    struct bt_le_adv_param adv_param = {
        .options = BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = BT_GAP_ADV_SLOW_INT_MIN,
        .interval_max = BT_GAP_ADV_SLOW_INT_MAX,
    };

    /* Lokale variabler for payload */
    uint16_t seq = 0;
    bool led_on = false;

    /* Første gang: legg company id først i buffer */
    sys_put_le16(COMPANY_ID_TEST, &mfg_buf[0]);

    /* Lag initial payload og kopier inn i bufferet etter company id */
    struct payload_v1 p;
    fill_payload(&p, seq, led_on);
    memcpy(&mfg_buf[2], &p, sizeof(p));

    /*
     * Advertising data (AD):
     *  - Flags: LE only
     *  - Fullt device name
     *  - Manufacturer data: vårt buffer (company id + payload)
     */
    const struct bt_data ad[] = {
        BT_DATA(BT_DATA_FLAGS, (uint8_t[]){ BT_LE_AD_NO_BREDR }, 1),
        BT_DATA(BT_DATA_NAME_COMPLETE,
                CONFIG_BT_DEVICE_NAME,
                sizeof(CONFIG_BT_DEVICE_NAME) - 1),
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_buf, sizeof(mfg_buf)),
    };

    /* Start advertising */
    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("bt_le_adv_start failed (%d)", err);
        return 0;
    }
    LOG_INF("Advertising started");

    /*
     * Hovedløkke:
     * - Oppdaterer payload hvert 2. sekund
     * - Toggler LED for synlig "heartbeat"
     * - Oppdaterer advertising data med bt_le_adv_update_data()
     */
    while (1) {
        k_sleep(K_SECONDS(2));

        seq++;
        led_on = !led_on;

        if (led.port) {
            gpio_pin_set_dt(&led, (int)led_on);
        }

        /* Oppdater payload og kopier inn i manufacturer buffer */
        fill_payload(&p, seq, led_on);
        memcpy(&mfg_buf[2], &p, sizeof(p));

        /*
         * Oppdater advertising data uten å stoppe advertising.
         * Hvis dette feiler kan man alternativt stoppe og starte advertising på nytt,
         * men update_data er "penere" når det fungerer.
         */
        err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            LOG_WRN("adv_update failed (%d) - continuing", err);
        }

        LOG_INF("tx seq=%u led=%d", seq, led_on);
    }
}
