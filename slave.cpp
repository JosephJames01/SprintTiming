#include <RH_RF69.h>
#include <SPI.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 8, 6, 9, 10, 5); // RS, E, D4, D5, D6, D7

#define RFM69_CS 4
#define RFM69_INT 3
#define RFM69_RST 2
#define SYNC_LED 13
#define LASER_PIN A5
#define DEAD_ZONE_US 

RH_RF69 rf69(RFM69_CS, RFM69_INT);
uint32_t baseTime = 0;
bool synchronized = false;
uint32_t prevMasterTime = 0;

// Sync status
bool showR = false;
int32_t errorBuffer[5] = {0};
uint8_t errorIndex = 0;

// Laser measurement
int baselineLaser = 512;
unsigned long lastBaselineUpdate = 0;
bool beamBroken = false;
uint32_t beamBreakTime = 0;
uint32_t eventTimestamp = 0;
uint32_t evnt = 0;
uint32_t event= 0;
int32_t reactionTime = 0;

void updateLCDState(int32_t currentError) {
  // Update error buffer
  errorBuffer[errorIndex] = abs(currentError);
  errorIndex = (errorIndex + 1) % 5;

  // Calculate average every 5 syncs
  if(errorIndex == 0) {
    int32_t avgError = 0;
    for(uint8_t i=0; i<5; i++) avgError += errorBuffer[i];
    showR = (avgError / 5) < 1000;
  }

  // Update display
  lcd.setCursor(15, 0);
  showR ? lcd.print('R') : lcd.print(' ');
}

void handleLaser() {
  const int currentLaser = analogRead(LASER_PIN);
  
  // Update baseline every 5 seconds with diagnostics
  if(millis() - lastBaselineUpdate > 5000) {
    baselineLaser = currentLaser;
    lastBaselineUpdate = millis();
    // Serial.print("[LASER] New baseline: ");
    // Serial.print(baselineLaser);
    // Serial.print(" | Current: ");
    // Serial.println(currentLaser);
  }

  // Detect beam break
  if(!beamBroken && currentLaser < (baselineLaser - 7)) {
    beamBroken = true;
    beamBreakTime = micros() - baseTime;
    
    if(eventTimestamp != 0  && event -1 == evnt ) {
      reactionTime = beamBreakTime - eventTimestamp;
   
      
      // Serial.print("[EVENT] Reaction: ");
      // Serial.print(reactionTime / 1000.0, 3);
      // Serial.println("ms");
      evnt ++;
    }
  }
  else if(beamBroken && currentLaser >= (baselineLaser - 7)) {
    beamBroken = false;
  }
}

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  pinMode(SYNC_LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  
  // Radio init
  digitalWrite(RFM69_RST, HIGH); delay(10);
  digitalWrite(RFM69_RST, LOW); delay(10);
  if(!rf69.init() || !rf69.setFrequency(868.0)) {
    lcd.print("Radio Fail!");
    Serial.println("Radio Fail!");
    while(1);
  }
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  // Example configuration for ~1.2kbps (long range)
  // Configure registers sequentially
rf69.spiWrite(RH_RF69_REG_02_DATAMODUL, 0x00);   // FSK, no shaping
rf69.spiWrite(RH_RF69_REG_03_BITRATEMSB, 0x02);  // Bitrate MSB
rf69.spiWrite(RH_RF69_REG_04_BITRATELSB, 0x8A);  // Bitrate LSB (1.2kbps)
rf69.spiWrite(RH_RF69_REG_05_FDEVMSB, 0x00);     // Freq deviation MSB
rf69.spiWrite(RH_RF69_REG_06_FDEVLSB, 0xD5);     // Freq deviation LSB (~5kHz)
rf69.spiWrite(RH_RF69_REG_19_RXBW, 0x40);        // RX bandwidth 10.4kHz
rf69.spiWrite(RH_RF69_REG_1A_AFCBW, 0x40);       // AFC bandwidth 10.4kHz
rf69.setModeRx();
  
  
  // Initial display
  lcd.print(" Initializing...");
  baselineLaser = analogRead(LASER_PIN);
  Serial.println("[SYSTEM] Slave initialized");
}

void loop() {
  static uint32_t lastStatus = 0;
  
  if(rf69.available()) {
    uint8_t buf[5];
    uint8_t len = sizeof(buf);
    
    if(rf69.recv(buf, &len)) {
      if(buf[0] == 'S') {
        // Handle sync packet
        digitalWrite(SYNC_LED, HIGH);
        uint32_t masterTime = ((uint32_t)buf[1] << 24) | 
                             ((uint32_t)buf[2] << 16) |
                             ((uint32_t)buf[3] << 8) |
                             buf[4];
        uint32_t slaveRxTime = micros();
        uint32_t elapsedTime = slaveRxTime - baseTime;

        if(!synchronized) {
          baseTime = slaveRxTime - masterTime - 1500;
          synchronized = true;
          lcd.clear();
          lcd.print("  Sync  ");
          Serial.println("[SYNC] Initial sync complete");
        } 
        else {
          int32_t error = masterTime - (slaveRxTime - baseTime);
          
           //Print sync diagnostics
           
           Serial.print("[SYNC] Elapsed: ");
          Serial.print(elapsedTime);
           Serial.print("μs | Error: ");
          Serial.print(error);
           Serial.println("μs");
           lcd.clear();
           lcd.print(error);
            lcd.setCursor(12, 1); 
            lcd.print("B:"); 
            lcd.print(baselineLaser);
             lcd.setCursor(0, 1);
            lcd.print("T:");
            lcd.print(reactionTime / 1000000.0, 3);
            lcd.print("s  ");
           
          
          if(abs(error) ) {
            static float integral = 0;
            integral = constrain(integral + error * 0.04, -100000, 100000);
            baseTime -= (error * 0.6) + integral;
          }
          
          updateLCDState(error);
        }
        digitalWrite(SYNC_LED, LOW);
      }
      else if(buf[0] == 'E') {
        // Store event timestamp
        eventTimestamp = ((uint32_t)buf[1] << 24) | 
                        ((uint32_t)buf[2] << 16) |
                        ((uint32_t)buf[3] << 8) |
                        buf[4];
        Serial.print("[EVENT] Received timestamp: ");
        Serial.println(eventTimestamp);
       
        event ++;
        //Timestamp Insurance 
        uint8_t response[5];
        response[0] = 'E';  
        response[1] = (eventTimestamp >> 24) & 0xFF;
        response[2] = (eventTimestamp >> 16) & 0xFF;
        response[3] = (eventTimestamp >> 8) & 0xFF;
        response[4] = eventTimestamp & 0xFF;

        // Send response
        rf69.send(response, sizeof(response));
        rf69.waitPacketSent();



      }
    }
  }

  handleLaser();



}