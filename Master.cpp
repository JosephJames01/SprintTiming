#include <RH_RF69.h>
#include <SPI.h>

#define RFM69_CS 4
#define RFM69_INT 3
#define RFM69_RST 2
#define SYNC_LED 13
#define SYNC_INTERVAL 2000000     // 2 seconds
#define BUTTON_PIN 5              // MUST BE PORTD PIN (D0-D7)
#define AUDIO_PIN 6
#define AMP_SHUTDOWN 8
#define DEBOUNCE_MICROS 20000     // 20ms debounce time

RH_RF69 rf69(RFM69_CS, RFM69_INT);
volatile bool buttonPressed = false;
volatile uint32_t buttonPressTime = 0;
bool sequenceActive = false;
uint32_t sequenceStart = 0;
uint32_t randomDelay = 0;
uint32_t eventTimestamp = 0;
bool setSoundPlayed = false;
bool waitingForRandomDelay = false;
bool sendingEvent = false;
unsigned long lastSendTime = 0;
const unsigned long RETRY_INTERVAL = 1000; // 1 second
bool ackReceived = false;
uint8_t eventPacket[5];

void setup() {
  Serial.begin(115200);
  pinMode(SYNC_LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(AUDIO_PIN, OUTPUT);
  pinMode(AMP_SHUTDOWN, OUTPUT);
  
  // Radio initialization
  digitalWrite(RFM69_RST, HIGH); delay(10);
  digitalWrite(RFM69_RST, LOW); delay(10);
  digitalWrite(AMP_SHUTDOWN, LOW);
  
  if (!rf69.init() || !rf69.setFrequency(868.0)) {
    Serial.println("Radio init failed!"); while(1);
  }
  rf69.setTxPower(20, true);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

// Configure registers sequentially
rf69.spiWrite(RH_RF69_REG_02_DATAMODUL, 0x00);   // FSK, no shaping
rf69.spiWrite(RH_RF69_REG_03_BITRATEMSB, 0x02);  // Bitrate MSB
rf69.spiWrite(RH_RF69_REG_04_BITRATELSB, 0x8A);  // Bitrate LSB (1.2kbps)
rf69.spiWrite(RH_RF69_REG_05_FDEVMSB, 0x00);     // Freq deviation MSB
rf69.spiWrite(RH_RF69_REG_06_FDEVLSB, 0xD5);     // Freq deviation LSB (~5kHz)
rf69.spiWrite(RH_RF69_REG_19_RXBW, 0x40);        // RX bandwidth 10.4kHz
rf69.spiWrite(RH_RF69_REG_1A_AFCBW, 0x40);       // AFC bandwidth 10.4kHz

  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT21);
}

void loop() {
  static uint32_t lastSync = 0;
  
  // Handle sync packets
  if (micros() - lastSync >= SYNC_INTERVAL) {
    sendSyncPacket();
    lastSync = micros();
    Serial.println("sent");
  }

  // Start sequence on button press
  if (buttonPressed && !sequenceActive) {
    sequenceActive = true;
    setSoundPlayed = false;
    waitingForRandomDelay = false;
    sequenceStart = millis();
    randomDelay = random(1700, 2401);
    buttonPressed = false;
     Serial.println("in");
     
  }

  // Sequence state machine
  if (sequenceActive) {
    uint32_t elapsed = millis() - sequenceStart;
    
    // 5s initial delay
    if (!setSoundPlayed && elapsed >= 5000) {
       
      playSound(1500, 200, 110);  // "set" sound
      setSoundPlayed = true;
      waitingForRandomDelay = true;
    }
    
    // Random delay period
    if (waitingForRandomDelay && elapsed >= 5000 + randomDelay) {
      eventTimestamp = micros();
      playSound(700, 255, 110);   // "starter" sound
      startEventSend();
      sequenceActive = false;
      digitalWrite(AMP_SHUTDOWN, LOW);

    }
  }
  sendEventPacket();
}

// Optimized ISR with debouncing
ISR(PCINT2_vect) {
  static uint32_t lastValidPress = 0;
  static bool lastState = HIGH;
  
  bool currentState = PIND & (1 << PIND5);
  
  // Only trigger on falling edge (button press)
  if (!currentState && lastState) {
    uint32_t now = micros();
    if (now - lastValidPress >= DEBOUNCE_MICROS) {
      buttonPressed = true;
      buttonPressTime = now;
      lastValidPress = now;
       Serial.println("press");
    }
  }
  lastState = currentState;
}

void sendSyncPacket() {
  digitalWrite(SYNC_LED, HIGH);
  uint32_t currentTime = micros();
  uint8_t packet[5] = {
    'S',
    (uint8_t)(currentTime >> 24),
    (uint8_t)(currentTime >> 16),
    (uint8_t)(currentTime >> 8),
    (uint8_t)currentTime
  };
  rf69.send(packet, sizeof(packet));
  rf69.waitPacketSent();
  rf69.setModeRx();
  digitalWrite(SYNC_LED, LOW);
}



void startEventSend() {
  eventPacket[0] = 'E';
  eventPacket[1] = (uint8_t)(eventTimestamp >> 24);
  eventPacket[2] = (uint8_t)(eventTimestamp >> 16);
  eventPacket[3] = (uint8_t)(eventTimestamp >> 8);
  eventPacket[4] = (uint8_t)eventTimestamp;
  sendingEvent = true;
  ackReceived = false;
  lastSendTime = 0; 
  Serial.println("done");
}


void sendEventPacket() {
  if (!sendingEvent) return; // Nothing to do

  // Send packet if never sent or retry interval elapsed
  if (lastSendTime == 0 || (millis() - lastSendTime >= RETRY_INTERVAL)) {
    rf69.send(eventPacket, sizeof(eventPacket));
    rf69.waitPacketSent();
    rf69.setModeRx();
    lastSendTime = millis();
  }

  // Check for ACK
  if (rf69.available()) {
    uint8_t buf[5];
    uint8_t len = sizeof(buf);
    Serial.print("available");
    if (rf69.recv(buf, &len)) {
      Serial.print("here");
      if (buf[0] == 'A') { // ACK identifier
        ackReceived = true;
        sendingEvent = false; 
        Serial.println("recieved!");
       
      }
    }
  }


}



void playSound(unsigned int frequency, int volume, unsigned long durationMs) {
  digitalWrite(AMP_SHUTDOWN, HIGH);
  unsigned long start = millis();
  unsigned long halfPeriod = 500000 / frequency;
  
  while (millis() - start < durationMs) {
    analogWrite(AUDIO_PIN, volume);
    delayMicroseconds(halfPeriod);
    analogWrite(AUDIO_PIN, 0);
    delayMicroseconds(halfPeriod);
  }
  analogWrite(AUDIO_PIN, 0);
}
