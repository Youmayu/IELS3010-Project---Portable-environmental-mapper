#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

/* --------------------------------------------------------------------
 * BLE protocol definitions
 * -------------------------------------------------------------------- */
#define COMPANY_ID   0xBEEF
#define SCAN_TIME_S  2   // Scan duration in seconds

/* --------------------------------------------------------------------
 * BLE scanner instance
 * -------------------------------------------------------------------- */
static BLEScan *scanner;

/* --------------------------------------------------------------------
 * Advertising callback
 * -------------------------------------------------------------------- */
class AdvertisementHandler : public BLEAdvertisedDeviceCallbacks {

  void onResult(BLEAdvertisedDevice device) override {

    if (!device.haveManufacturerData()) {
      return;
    }

    std::string rawData = device.getManufacturerData();
    const uint8_t *data = (const uint8_t *)rawData.data();

    if (rawData.length() < 7) {
      return;
    }

    uint16_t companyId = data[0] | (data[1] << 8);
    if (companyId != COMPANY_ID) {
      return;
    }

    uint8_t  teamId      = data[2];
    uint16_t sequence    = data[3] | (data[4] << 8);
    int16_t  temperature = data[5] | (data[6] << 8);

    Serial.print("RSSI=");
    Serial.print(device.getRSSI());
    Serial.print(" dBm  ");

    Serial.print("Team=");
    Serial.print(teamId);

    Serial.print("  Seq=");
    Serial.print(sequence);

    Serial.print("  Temp=");
    Serial.print(temperature / 100);
    Serial.print(".");
    Serial.print(abs(temperature % 100));
    Serial.println(" C");
  }
};

/* --------------------------------------------------------------------
 * Setup
 * -------------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 BLE scanner started");

  BLEDevice::init("");

  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new AdvertisementHandler());
  scanner->setActiveScan(true);

  /* Continuous scan window to catch slow advertisers */
  scanner->setInterval(100);
  scanner->setWindow(100);
}

/* --------------------------------------------------------------------
 * Main loop
 * -------------------------------------------------------------------- */
void loop() {
  /* Scan for a short period, then restart */
  scanner->start(SCAN_TIME_S, false);
  delay(100);
}
