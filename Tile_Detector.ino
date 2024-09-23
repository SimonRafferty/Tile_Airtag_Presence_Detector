//************************************************************************************** 
// Board XIAO ESP32C6
// Simon Rafferty 2024
//
// This is a presence detector based on Tile BLE Tags (but it could 
// just as easily work with Airtags).
//
// Taking D0 Low puts it into scanning mode.  Tags are sorted by decreasing
// signal strength.  When D0 goes High, the tag with the strongest signal is stored.
// Normally, at power-up, it scans for the stored tag.  If detected, flash the LED
// and make D1 High.  After 30 seconds of no contact with the tag, D1 goes low.
//
// I'm using this as a kind of valet mode for my DIY Electric vehicle.
// If my tag is detected, it enables full speed & power.  If not, speed is limited to
// 10mph with unexciting acceleration.
//
//************************************************************************************** 




#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>
#include <vector>

// Structure to hold device information
struct DeviceInfo {
  String macAddress;
  String deviceName;
  int rssi;
};

BLEScan* pBLEScan;
Preferences preferences;

const int D0_PIN = D0;           // Adjust as needed
const int D1_PIN = D1;           // Adjust as needed
const int LED_PIN = LED_BUILTIN; // Adjust as needed

std::vector<DeviceInfo> devices;
String storedMacAddress = "";
bool deviceDetected = false;

int prevD0State = HIGH;

// Enumeration for operation modes
enum Mode {
  SCAN_FOR_STORED_DEVICE,
  SCAN_AND_LIST_DEVICES
};

Mode currentMode;

// Flag to indicate whether scanning is in progress
bool isScanning = false;

// Time when the stored device was last detected
unsigned long lastDetectedTime = 0;

// Callback class to handle discovered devices
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    String deviceName = advertisedDevice.getName().c_str();
    String macAddress = advertisedDevice.getAddress().toString().c_str();
    int rssi = advertisedDevice.getRSSI();

    if (deviceName == "Tile") {
      // If in SCAN_AND_LIST_DEVICES mode, collect devices
      if (currentMode == SCAN_AND_LIST_DEVICES) {
        // Check if device is already in list
        bool found = false;
        for (auto &device : devices) {
          if (device.macAddress == macAddress) {
            found = true;
            // Update RSSI if necessary
            if (rssi > device.rssi) {
              device.rssi = rssi;
            }
            break;
          }
        }
        if (!found) {
          DeviceInfo device = {macAddress, deviceName, rssi};
          devices.push_back(device);
        }
      }

      // If in SCAN_FOR_STORED_DEVICE mode, check for stored MAC address
      if (currentMode == SCAN_FOR_STORED_DEVICE && storedMacAddress != "" && macAddress == storedMacAddress) {
        // Update last detected time
        lastDetectedTime = millis();

        if (!deviceDetected) {
          Serial.println("Stored device detected!");
          Serial.print("MAC Address: ");
          Serial.println(macAddress);
          Serial.print("Device Name: ");
          Serial.println(deviceName);
          Serial.print("RSSI: ");
          Serial.println(rssi);

          // Flash LED continuously and set D1 HIGH
          digitalWrite(D1_PIN, HIGH);
          deviceDetected = true;
        }
      }
    }
  }
};

void flashLED() {
  static unsigned long lastToggleTime = 0;
  static bool ledState = false;

  unsigned long currentTime = millis();
  if (currentTime - lastToggleTime >= 500) { // Toggle every 500 ms
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastToggleTime = currentTime;
  }
}

void scanComplete(BLEScanResults scanResults) {
  isScanning = false;
  pBLEScan->clearResults(); // Clear scan results to free memory

  if (currentMode == SCAN_AND_LIST_DEVICES) {
    // Sort devices by RSSI (signal strength)
    if (!devices.empty()) {
      std::sort(devices.begin(), devices.end(), [](const DeviceInfo &a, const DeviceInfo &b) {
        return a.rssi > b.rssi;
      });

      // Print devices
      Serial.println("Devices found:");
      for (const auto &device : devices) {
        Serial.print("MAC: ");
        Serial.print(device.macAddress);
        Serial.print(" Name: ");
        Serial.print(device.deviceName);
        Serial.print(" RSSI: ");
        Serial.println(device.rssi);
      }
    } else {
      Serial.println("No devices named 'Tile' found.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Device starting up...");

  // Initialize GPIO pins
  pinMode(D0_PIN, INPUT_PULLUP); // D0 as input with pull-up resistor
  pinMode(D1_PIN, OUTPUT);
  digitalWrite(D1_PIN, LOW);     // Initially low
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize preferences (non-volatile storage)
  preferences.begin("storage", false); // Namespace "storage", false for read/write
  storedMacAddress = preferences.getString("storedMac", "");
  if (storedMacAddress != "") {
    Serial.print("Stored MAC Address: ");
    Serial.println(storedMacAddress);
  } else {
    Serial.println("No MAC Address stored.");
  }

  // Determine initial mode based on D0 state at power-up
  int d0State = digitalRead(D0_PIN);
  prevD0State = d0State; // Initialize previous D0 state
  if (d0State == HIGH) {
    if (storedMacAddress != "") {
      Serial.println("D0 is HIGH on power-up: Scanning for the stored device...");
      currentMode = SCAN_FOR_STORED_DEVICE;
    } else {
      Serial.println("D0 is HIGH on power-up but no stored MAC Address found.");
    }
  } else {
    Serial.println("D0 is LOW on power-up: Scanning and listing devices...");
    currentMode = SCAN_AND_LIST_DEVICES;
  }

  // Initialize BLE
  Serial.println("Initializing BLE...");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // Active scan to receive more data
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);       // Must be less than or equal to interval
  Serial.println("BLE initialized.");
}

void loop() {
  int d0State = digitalRead(D0_PIN);

  // Detect D0 state change from LOW to HIGH
  if (prevD0State == LOW && d0State == HIGH) {
    Serial.println("D0 changed from LOW to HIGH.");
    if (!devices.empty()) {
      // Store the MAC address of the strongest device
      storedMacAddress = devices.front().macAddress;
      preferences.putString("storedMac", storedMacAddress);
      Serial.print("Stored MAC Address: ");
      Serial.println(storedMacAddress);
    } else {
      Serial.println("No devices to store.");
    }
    // Clear devices list
    devices.clear();
    // Switch to SCAN_FOR_STORED_DEVICE mode
    if (storedMacAddress != "") {
      currentMode = SCAN_FOR_STORED_DEVICE;
    }
  }

  prevD0State = d0State; // Update previous D0 state

  // Scanning logic
  if (!isScanning) {
    if (currentMode == SCAN_AND_LIST_DEVICES) {
      Serial.println("Scanning and listing devices named 'Tile'...");
      devices.clear(); // Clear previous devices list
      isScanning = true;
      pBLEScan->start(5, scanComplete); // Scan for 5 seconds
    } else if (currentMode == SCAN_FOR_STORED_DEVICE) {
      if (storedMacAddress != "") {
        Serial.println("Scanning for the stored device...");
        isScanning = true;
        pBLEScan->start(5, scanComplete); // Scan for 5 seconds
      } else {
        Serial.println("No stored MAC Address to scan for.");
      }
    }
  }

  // Check if the stored device has not been detected for 30 seconds
  if (deviceDetected && (millis() - lastDetectedTime >= 30000)) {
    Serial.println("Stored device not detected for 30 seconds.");
    deviceDetected = false;
    digitalWrite(D1_PIN, LOW);
    digitalWrite(LED_PIN, LOW); // Turn off the LED
  }

  // Flash the LED if the stored device is detected
  if (deviceDetected) {
    flashLED();
  }

  // Add a small delay to prevent watchdog timer resets
  delay(100);
}
