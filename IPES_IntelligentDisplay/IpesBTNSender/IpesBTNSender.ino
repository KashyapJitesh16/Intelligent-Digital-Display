#include <Arduino.h>
#include <ArduinoJson.h>

// Define GPIO pins connected to TTP223 'I/O' or 'OUT' pins
#define TTP_PIN_PWR    22   // TTP223 Module 1 - POWER
#define TTP_PIN_SET    19   // TTP223 Module 2 - SETTINGS
#define TTP_PIN_ZERO   18   // TTP223 Module 3 - ZERO
#define TTP_PIN_PEAK   21   // TTP223 Module 4 - PEAK
#define TTP_PIN_NAV    23   // TTP223 Module 5 - NAV (navigation)

// Hardware UART Pins for sending to Display ESP
#define TX_PIN         17
#define RX_PIN         16
#define UART_BAUD      115200

// Touch State Tracking
bool lastPwr  = false;
bool lastSet  = false;
bool lastZero = false;
bool lastPeak = false;
bool lastNav  = false;

void sendButtonJson(const char* btnName, const char* eventType) {
  StaticJsonDocument<128> doc;
  doc["button"] = btnName;   // "POWER","SETTINGS","ZERO","PEAK","NAV"
  doc["event"]  = eventType; // "PRESSED" or "RELEASED"

  String jsonOutput;
  serializeJson(doc, jsonOutput);

  // Send JSON string over UART with newline terminator
  Serial1.println(jsonOutput);

  // Debug output on main Serial Monitor
  Serial.print("Sent: ");
  Serial.println(jsonOutput);
}

void setup() {
  Serial.begin(115200);

  // Initialize UART1 for inter-ESP communication
  Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // Set TTP223 signal pins as Digital Inputs
  pinMode(TTP_PIN_PWR,  INPUT);
  pinMode(TTP_PIN_SET,  INPUT);
  pinMode(TTP_PIN_ZERO, INPUT);
  pinMode(TTP_PIN_PEAK, INPUT);
  pinMode(TTP_PIN_NAV,  INPUT);

  Serial.println("TTP223 Touch Button Transmitter Initialized (5 buttons).");
}

void loop() {
  // Read TTP223 digital outputs (HIGH = Touched, LOW = Released)
  bool curPwr  = digitalRead(TTP_PIN_PWR)  == HIGH;
  bool curSet  = digitalRead(TTP_PIN_SET)  == HIGH;
  bool curZero = digitalRead(TTP_PIN_ZERO) == HIGH;
  bool curPeak = digitalRead(TTP_PIN_PEAK) == HIGH;
  bool curNav  = digitalRead(TTP_PIN_NAV)  == HIGH;

  // Detect State Changes & Send JSON
  if (curPwr != lastPwr) {
    sendButtonJson("POWER", curPwr ? "PRESSED" : "RELEASED");
    lastPwr = curPwr;
  }
  if (curSet != lastSet) {
    sendButtonJson("SETTINGS", curSet ? "PRESSED" : "RELEASED");
    lastSet = curSet;
  }
  if (curZero != lastZero) {
    sendButtonJson("ZERO", curZero ? "PRESSED" : "RELEASED");
    lastZero = curZero;
  }
  if (curPeak != lastPeak) {
    sendButtonJson("PEAK", curPeak ? "PRESSED" : "RELEASED");
    lastPeak = curPeak;
  }
  if (curNav != lastNav) {
    sendButtonJson("NAV", curNav ? "PRESSED" : "RELEASED");
    lastNav = curNav;
  }

  //delay(30); // Software debounce delay
}
