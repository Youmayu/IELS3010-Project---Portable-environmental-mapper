#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#define COMPANY_ID 0xBEEF
#define PAYLOAD_LEN 7

static bool ad_parse_cb(struct bt_data *data, void *user_data)
{
    if (data->type != BT_DATA_MANUFACTURER_DATA) {
        return true;
    }

    if (data->data_len != PAYLOAD_LEN) {
        return true;
    }

    const uint8_t *p = data->data;

    uint16_t company_id = sys_get_le16(&p[0]);
    if (company_id != COMPANY_ID) {
        return true;
    }

    uint8_t team_id   = p[2];
    uint16_t seq      = sys_get_le16(&p[3]);
    int16_t temp_x100 = (int16_t)sys_get_le16(&p[5]);

    printk("{\"seq\":%u,\"temperature\":%.2f}\n", seq, temp_x100 / 100.0);

    return false;
}

static void scan_cb(const bt_addr_le_t *addr,
                    int8_t rssi,
                    uint8_t adv_type,
                    struct net_buf_simple *buf)
{
    bt_data_parse(buf, ad_parse_cb, NULL);
}

void main(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        printk("BLE init failed (%d)\n", err);
        return;
    }

    printk("BLE scanner started\n");

    struct bt_le_scan_param scan_param = {
        .type       = BT_HCI_LE_SCAN_PASSIVE,
        .options    = BT_LE_SCAN_OPT_NONE,
        .interval   = BT_GAP_SCAN_FAST_INTERVAL,
        .window     = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err) {
        printk("Scan start failed (%d)\n", err);
        return;
    }
}