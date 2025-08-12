// #include <Arduino.h>
// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>

// // BLE UUIDs
// #define SERVICE_UUID           "12345678-1234-1234-1234-1234567890ab"
// #define CHARACTERISTIC_UUID    "abcd1234-5678-90ab-cdef-1234567890ab"

// BLECharacteristic *pCharacteristic;
// bool deviceConnected = false;

// class MyServerCallbacks : public BLEServerCallbacks {
//   void onConnect(BLEServer* pServer) {
//     deviceConnected = true;
//     Serial.println("Device connected");
//   };

//   void onDisconnect(BLEServer* pServer) {
//     deviceConnected = false;
//     Serial.println("Device disconnected");
//     // Restart advertising so another device can connect
//     pServer->getAdvertising()->start();
//     Serial.println("Advertising restarted");
//   }
// };

// class MyCallbacks : public BLECharacteristicCallbacks {
//   void onWrite(BLECharacteristic *pChar) {
//     String value = pChar->getValue();
//     if (value.length() > 0) {
//       Serial.print("Received Value: ");
//       Serial.println(value);
//     }
//   }
// };


// void setup() {
//   Serial.begin(115200);

//   // Initialize BLE device
//   BLEDevice::init("ESP32_BLE_Server");

//   // Create BLE Server
//   BLEServer *pServer = BLEDevice::createServer();
//   pServer->setCallbacks(new MyServerCallbacks());

//   // Create BLE Service
//   BLEService *pService = pServer->createService(SERVICE_UUID);

//   // Create BLE Characteristic
//   pCharacteristic = pService->createCharacteristic(
//                       CHARACTERISTIC_UUID,
//                       BLECharacteristic::PROPERTY_READ   |
//                       BLECharacteristic::PROPERTY_WRITE  |
//                       BLECharacteristic::PROPERTY_NOTIFY
//                     );

//   pCharacteristic->setCallbacks(new MyCallbacks());

//   // Add descriptor for notifications
//   pCharacteristic->addDescriptor(new BLE2902());

//   // Set initial value
//   pCharacteristic->setValue("Hello from ESP32");

//   // Start the service
//   pService->start();

//   // Start advertising
//   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//   pAdvertising->addServiceUUID(SERVICE_UUID);
//   pAdvertising->setScanResponse(true);
//   pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
//   pAdvertising->setMinPreferred(0x12);
//   BLEDevice::startAdvertising();
//   Serial.println("Waiting for a client to connect...");
// }

// void loop() {
//   if (deviceConnected) {
//     static unsigned long lastNotifyTime = 0;
//     unsigned long now = millis();
//     if (now - lastNotifyTime > 5000) {
//       lastNotifyTime = now;
//       String newValue = "Time: " + String(now / 1000);
//       pCharacteristic->setValue(newValue.c_str());
//       pCharacteristic->notify();
//       Serial.print("Notified: ");
//       Serial.println(newValue);
//     }
//   }
//   delay(20);
// }









#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

// Button config
const int buttonPin = 4; // safe pin for ESP32-S3
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool startState = true;   // true = next press sends "start", false = "cancel"

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Buffers for message assembly and deduplication
static String receivedBuffer = "";
static String lastChunk = "";

// BLE Server Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("Device disconnected");
    // Restart advertising to allow new connections
    pServer->getAdvertising()->start();
    Serial.println("Advertising restarted");
  }
};

// BLE Characteristic Callbacks with deduplication
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String chunk = pChar->getValue();
    if (chunk.isEmpty()) return;

    // Skip duplicate chunk
    if (chunk == lastChunk) {
      Serial.print("Duplicate chunk ignored: ");
      Serial.println(chunk);
      return;
    }
    lastChunk = chunk;

    Serial.print("Chunk received: ");
    Serial.println(chunk);

    // Append chunk to buffer
    receivedBuffer += chunk;

    // Check for <EOM> delimiter indicating end of message
    int eomIndex = receivedBuffer.indexOf("<EOM>");
    while (eomIndex != -1) {
      String fullMessage = receivedBuffer.substring(0, eomIndex);

      Serial.print("Full message received: ");
      Serial.println(fullMessage);

      // Remove processed message + delimiter
      receivedBuffer.remove(0, eomIndex + 5);

      // Check for more messages
      eomIndex = receivedBuffer.indexOf("<EOM>");
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.println("Initializing ESP32 BLE...");

  BLEDevice::init("ESP32S3_Button");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setValue("Hello from ESP32");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x00);
  BLEDevice::startAdvertising();

  Serial.println("BLE ready, waiting for connections...");
}

void loop() {
  int reading = digitalRead(buttonPin);

  // Debounce logic
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) { // Button pressed
        if (deviceConnected) {
          if (startState) {
            Serial.println("Sending: start");
            pCharacteristic->setValue("start");
            pCharacteristic->notify();
            startState = false;
          } else {
            Serial.println("Sending: cancel");
            pCharacteristic->setValue("cancel");
            pCharacteristic->notify();
            startState = true;
          }
        } else {
          Serial.println("Button pressed but no device connected");
        }
      }
    }
  }

  lastButtonState = reading;

  delay(10); // Small delay for CPU relief
}
