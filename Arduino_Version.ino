#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>



// Buffer for full message assembly
static String receivedBuffer = "";
static String lastChunk = ""; // Track last chunk to skip duplicates



const int buttonPin = 4; // safe pin for ESP32-S3
int buttonState = HIGH;  // default HIGH because of pull-up
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

bool startState = true; // true = next press sends "start", false = next press sends "cancel"

// BLE settings
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

// Callback class to handle server events
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected!");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected!");
      // Restart advertising when disconnected
      pServer->startAdvertising();
      Serial.println("Started advertising again");
    }
};



class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        String chunk = pCharacteristic->getValue();
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

        // Look for <EOM> anywhere in buffer
        int eomIndex = receivedBuffer.indexOf("<EOM>");
        while (eomIndex != -1) {
            String fullMessage = receivedBuffer.substring(0, eomIndex);

            Serial.print("Full message received: ");
            Serial.println(fullMessage);

            // Remove processed message + delimiter from buffer
            receivedBuffer.remove(0, eomIndex + 5); // 5 = length of "<EOM>"

            // Look for another <EOM> in case multiple messages were queued
            eomIndex = receivedBuffer.indexOf("<EOM>");
        }
    }
};


void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.println("Initializing ESP32 BLE...");

  // BLE init
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
  
  // Set callbacks for both notifications and writes
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("BLE ready, waiting for connections...");
}

void loop() {
  int reading = digitalRead(buttonPin);

  // debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) { // button pressed
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
  
  // Small delay to prevent excessive CPU usage
  delay(10);
}
