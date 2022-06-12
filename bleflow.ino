#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <WiFi.h>

// Adjust the ratio with a calibration pour and a serial connection.
// const int RATIO=5.2; //narrow feed seems to need lower.
const int RATIO = 7.055; // factory data

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID BLEUUID("9b69916a-dc6b-11ec-9d64-0242ac120002") // UART service UUID

//Not actually used yet
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) { 
    Serial.println("Bluetooth connected");
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Bluetooth disconnected");
    deviceConnected = false;
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
  }
};


byte statusLed = 10;

uint16_t pulseCount = 0;
uint32_t totalMilliLitres;
unsigned long oldTime;

gpio_num_t sensorPin;
byte sensorInterrupt;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static int intcount = 0;
void IRAM_ATTR pulseCounter() {
  portENTER_CRITICAL_ISR(&mux);
  // Increment the pulse counter
  pulseCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  esp_err_t err;
  disableCore0WDT();

  BLEBegin(); // Initialize Bluetooth
  WiFi.mode(WIFI_OFF);

  sensorPin = GPIO_NUM_0;
  sensorInterrupt = digitalPinToInterrupt(sensorPin); // 0 = digital pin 2

  // Initialize a serial connection for reporting values to the host
  Serial.begin(9600);

  // Set up the status LED line as an output
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH); // We have an active-low LED attached

  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, LOW);

  pulseCount = 0;
  totalMilliLitres = 0;
  oldTime = 0;

  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  Serial.print("Started");
}

BLEAdvertising *pAdvertising;
BLEService *pService;

void BLEBegin() {
  // Create the BLE Device
  BLEDevice::init(/*BLE name*/ "Flow Service");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service - in case we want to provide a full service.
  pService = pServer->createService(SERVICE_UUID);

  // Start the service
  pService->start();

  BLEAdvertisementData advertisementData;

  advertisementData.setName("Flow Service");
  advertisementData.setManufacturerData("0.0");

  pAdvertising = pServer->getAdvertising();
  pAdvertising->setAdvertisementData(advertisementData);
  pAdvertising->addServiceUUID(BLEUUID(pService->getUUID()));
  pAdvertising->start();
}

/**
 * Main program loop
 */
struct Data {
  uint16_t volume; // ml up to 65l
  uint16_t pulse;
};

void lightSleep() {
  gpio_wakeup_disable(sensorPin);
  detachInterrupt(sensorInterrupt);

  digitalWrite(statusLed, LOW);
  int level = digitalRead(sensorPin);
  // wake on change
  esp_sleep_enable_timer_wakeup(15 * 1000000); // idle update every 15s
  gpio_wakeup_enable(sensorPin,
                     (level == 0) ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  esp_light_sleep_start();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_GPIO) {
    // pulseCount++; //might lose a tick on power?
  }

  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  digitalWrite(statusLed, HIGH);
}

void loop() {
  esp_err_t err;

  static uint16_t lastCount = 0;

  if ((millis() - oldTime) > 1000) // 1 second tick while awake
  {
    oldTime = millis();

    if (lastCount == pulseCount) { // no ticks this second, back to sleep
      Serial.print("sleep ");
      lightSleep();
      Serial.print("wake ");
    }

    portENTER_CRITICAL(&mux);
    lastCount = pulseCount;
    portEXIT_CRITICAL(&mux);

    totalMilliLitres = lastCount / RATIO;

    unsigned int frac;
    Serial.print("Count: ");
    Serial.println(lastCount);

    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.println("mL");

    Data data;
    data.pulse = lastCount;
    data.volume = totalMilliLitres;

    char tmp[10];
    sprintf(tmp, "%d", totalMilliLitres);

    BLEAdvertisementData advertisementData;
    advertisementData.setName("Flow Service");

    pAdvertising->stop();
    advertisementData.setManufacturerData(tmp);
    pAdvertising->setAdvertisementData(advertisementData);
    pAdvertising->start();

    // In case we want to use a BLE characteristic instead
    // pTxCharacteristic->setValue((uint8_t*)&data, sizeof(uint32_t));
    // pTxCharacteristic->notify();
  }
}
