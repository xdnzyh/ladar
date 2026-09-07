#include <Arduino.h>
#include <esp_timer.h>

HardwareSerial radio(2);
constexpr int RELAY = 13;
constexpr int SENSOR = 27;
constexpr int AUX = 26;
constexpr uint32_t CAPACITY = 32;
struct Trigger { uint64_t time_us; uint32_t count; };
volatile Trigger triggers[CAPACITY];
volatile uint32_t head = 0, tail = 0, count = 0;
volatile bool enabled = false, overflow = false;
volatile uint64_t motorStart = 0, lastEdge = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
String session, incoming;
uint64_t lastCommand = 0;
struct Outgoing { String text; uint64_t received; bool sync; };
Outgoing outgoing[CAPACITY];
uint32_t outHead = 0, outTail = 0;

void motorOff() {
  portENTER_CRITICAL(&mux);
  enabled = false;
  portEXIT_CRITICAL(&mux);
  digitalWrite(RELAY, LOW);
}

void ARDUINO_ISR_ATTR onEdge() {
  const uint64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&mux);
  if (enabled && now - motorStart >= 300000 && now - lastEdge >= 8000) {
    lastEdge = now;
    ++count;
    const uint32_t next = (head + 1) % CAPACITY;
    if (next == tail) overflow = true;
    else {
      triggers[head].time_us = now;
      triggers[head].count = count;
      head = next;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void enqueue(const String &text, bool sync = false, uint64_t received = 0) {
  const uint32_t next = (outHead + 1) % CAPACITY;
  if (next == outTail) {
    motorOff();
    outHead = outTail = 0;
    outgoing[outHead] = {"ERROR TX_OVERFLOW", 0, false};
    outHead = 1;
    return;
  }
  outgoing[outHead] = {text, received, sync};
  outHead = next;
}

void command(String line, uint64_t received) {
  line.trim();
  if (!line.length()) return;
  lastCommand = received;
  if (line == "OFF") {
    motorOff();
    session = "";
    outHead = outTail = 0;
    enqueue("OK OFF");
  } else if (line == "PING") enqueue("PONG");
  else if (line == "STATUS") {
    enqueue("STATUS MOTOR=" + String(enabled ? 1 : 0) + " COUNT=" + String(count));
  } else if (line.startsWith("SYNC ")) {
    enqueue(line.substring(5), true, received);
  } else if (line.startsWith("ROT ") && line.length() > 4) {
    motorOff();
    session = line.substring(4);
    outHead = outTail = 0;
    portENTER_CRITICAL(&mux);
    head = tail = count = 0;
    overflow = false;
    lastEdge = 0;
    motorStart = esp_timer_get_time();
    enabled = true;
    portEXIT_CRITICAL(&mux);
    digitalWrite(RELAY, HIGH);
    enqueue("OK ROT " + session);
  } else enqueue("ERROR COMMAND");
}

void setup() {
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);
  pinMode(SENSOR, INPUT_PULLUP);
  pinMode(AUX, INPUT_PULLUP);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);
  digitalWrite(33, LOW);
  digitalWrite(32, LOW);
  radio.begin(115200, SERIAL_8N1, 16, 17);
  attachInterrupt(digitalPinToInterrupt(SENSOR), onEdge, FALLING);
  enqueue("READY ROTATION_SYNC_V1");
}

void loop() {
  while (radio.available()) {
    const char c = radio.read();
    if (c == '\n') {
      command(incoming, esp_timer_get_time());
      incoming = "";
    } else if (c != '\r') incoming += c;
    if (incoming.length() > 256) {
      incoming = "";
      motorOff();
      enqueue("ERROR RX_OVERFLOW");
    }
  }
  Trigger event;
  bool available = false, lost = false;
  portENTER_CRITICAL(&mux);
  lost = overflow;
  overflow = false;
  if (tail != head) {
    event.time_us = triggers[tail].time_us;
    event.count = triggers[tail].count;
    tail = (tail + 1) % CAPACITY;
    available = true;
  }
  portEXIT_CRITICAL(&mux);
  if (lost) {
    motorOff();
    enqueue("ERROR IRQ_OVERFLOW");
  }
  if (available && enabled) {
    char message[128];
    snprintf(message, sizeof(message), "TRIG %s %lu %llu", session.c_str(),
             static_cast<unsigned long>(event.count), static_cast<unsigned long long>(event.time_us));
    enqueue(message);
  }
  if (enabled && esp_timer_get_time() - lastCommand > 40000000) {
    motorOff();
    enqueue("ERROR WATCHDOG");
  }
  if (outTail != outHead && digitalRead(AUX) && radio.availableForWrite() >= 120) {
    Outgoing &item = outgoing[outTail];
    String message = item.text;
    if (item.sync) {
      char buffer[160];
      snprintf(buffer, sizeof(buffer), "SYNC %s %llu %llu", item.text.c_str(),
               static_cast<unsigned long long>(item.received),
               static_cast<unsigned long long>(esp_timer_get_time()));
      message = buffer;
    }
    radio.print(message + "\r\n");
    outTail = (outTail + 1) % CAPACITY;
  }
  delay(1);
}
