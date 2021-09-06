#include <SPI.h>

#include "MPU6000.h"

#include "APM_PPM.h"
#include "APM2RCOutput.h"

#include "COBS.h"

//LED output pins
const int ledBlue = 25;
const int ledYellow = 26;
const int ledRed = 27;

//SPI Bus SlaveSelect pins
const int imuSelect = 53;
const int pressureSelect = 40;

/*
  _______________DataFlash(AT45DB161-MU) Pinout_______________
  PJ1 = MOSI_DF = 14
  PJ2 = SCK_DF = NA
  PA6 = CS_DF = 28
  PG0 = RST_DEF = 41
  I assume these pins go to some SPI hardware...
*/

/*
  _______________Magnetometer(HMC5883L) Pinout________________
  SCL = PD0 = 21
  SDA = PD1 = 20
  This seems like the defualt bus
*/

/*
  _______________Servo PPM Encoder Pinout______________________
  Definitely a lot more going on in the circuit than I orignally thought.
  ATMEGA32U4  ATMEGA2560
  PPM_PD1      PA2
  PPM_PD0      PA1

  PPM_PC2 => Throws switch for whole system
  This pin is on the ATMEGA32U4

  This will throw the connections for pins

  RX0-1                     ____PPM_TX (Normally Open)
  ATMEGA2560_PE0 (RX?)_____|____3DR_RX (Normally Closed)

  TX0-O                     ____PPM_RX (Normally Open)
  ATMEGA2560_PE1 (TX?)_____|____3DR_TX (Normally Closed)
*/

APM_PPM rxin;

/*
  _______________Extra IO_______________________________________
  IO1 = PF0 = ADC0
  IO2 = PF1 = ADC1
  IO3 = PF2 = ADC2
  IO4 = PF3 = ADC3
  IO5 = PF4 = ADC4
  IO6 = PF5 = ADC5
  IO7 = PF6 = ADC6
  IO8 = PF8 = ADC7
  IO9 = PK0 = ADC8
*/
const int io_count = 9;
const int io_pins[9] = {A0, A1, A2, A3, A4, A5, A6, A7, A8};

APM2RCOutput output;

//SPI Bus SlaveSelect pins
MPU6000 imu;
float accelVec[3];
float gyroVec[3];

#define X 0
#define Y 1
#define Z 2

void setup () {

  Serial.begin(115200);

  pinMode(ledBlue, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledRed, OUTPUT);

  digitalWrite(ledBlue, HIGH);
  digitalWrite(ledYellow, HIGH);
  digitalWrite(ledRed, HIGH);

  pinMode(imuSelect, OUTPUT);
  pinMode(pressureSelect, OUTPUT);

  digitalWrite(imuSelect, HIGH);
  digitalWrite(pressureSelect, HIGH);

  imu.initialize();
  
  output.init();
  for (int i = 0; i < 8; i++) {
    output.enable_ch(i);
  }

  rxin.initialize();
}

class Rate {
  public:
  Rate(uint32_t target_us);
  void tick();
  private:
  uint32_t target_us;
  uint32_t last_us;
};

Rate::Rate(uint32_t target_us) : target_us(target_us), last_us(0) {}

void Rate::tick() {
  //Serial.println("");
  //Serial.println(last_us);
  digitalWrite(ledBlue, 1); // Turn off blue when sleeping
  while ((micros() - last_us) < target_us) {
    delay(0);
  }
  //Serial.println(micros() - last_us);
  last_us = micros();
  digitalWrite(ledBlue, 0); // Turn on blue when in action
}

Rate loop_rate(10000);

class SerialServo {
  public:
  SerialServo();
  void update_from_serial();
  void update_output();
  bool is_enabled();
  private:
  uint32_t last_update;
  bool enabled;
};

SerialServo::SerialServo() : last_update(0), enabled(false) {}

bool SerialServo::is_enabled() {
  return enabled;
}

void SerialServo::update_from_serial() {
  int ch = Serial.read();
  if (ch == -1) {
    return;
  }
  uint8_t buf[32];
  size_t buflen = 0;
  size_t max_skip = 20;
  bool has_packet = false;
  uint8_t buf_head[4];
  bool first_loop = true;
  while(max_skip > 0) {
    if (! first_loop) {
      // Prevent the first byte read above from overwritten
    ch = Serial.read();
    }
    first_loop = false;
    if (ch == -1) {
      max_skip -= 1;
      continue;
    }
    if (buflen < 4) {
      buf_head[0] = buf_head[1];
      buf_head[1] = buf_head[2];
      buf_head[2] = buf_head[3];
      buf_head[3] = ch;
      if (buf_head[1] == 'O' && buf_head[2] == 'U' && buf_head[3] == 'T') {
        memcpy(buf, buf_head, 4);
        buflen = 4;
      }
      continue;
    }
    buf[buflen] = ch;
    buflen ++;
    if (ch == 0) {
      has_packet = true;
      break;
    }
    if (buflen >= 32) {
      return;
    }
  }
  if (! has_packet) {
    return;
  }
  uint8_t buf2[32];
  
 
  size_t len2 = COBSDecode(buf, buflen, buf2, 32);
  if (len2 <= 0) {
    // Bad parse
    return;
  }
  
  uint16_t * outputs = (uint16_t *)(buf2 + 3);

  for (int i = 0; i < 8; i++) {
    if (outputs[i] > 3000) {
      // Shortcut to disable servo
      output.disable_ch(i);
    } else {
      
      output.enable_ch(i);
      output.write(i, outputs[i]);
    }
  }
  
  enabled = true;
  last_update = millis();
}

void SerialServo::update_output() {
  // Disable if timeout
  if (millis() - last_update > 100) {
    for (int i = 0; i < 8; i++) {
    output.disable_ch(i);
    }
    enabled = false;
  }
}

SerialServo ss;

void loop() {
  loop_spi_print();
  loop_ppm_print();
  ss.update_from_serial();
  ss.update_output();
  digitalWrite(ledYellow, !ss.is_enabled());
  loop_rate.tick();
}

void loop_ppm_print () {
  uint8_t buf[32], buf2[32];
  buf[0] = 'P';
  buf[1] = 'P';
  buf[2] = 'M';
  buf[3] = rxin.newData();
  rxin.read((uint16_t *)(buf+4));
  size_t buflen = COBSEncode((uint8_t *) buf, 4 + (numChannel * 2), buf2, 32);
  Serial.write(buf2, buflen);
}

void loop_spi_print () {
  int16_t dofs[6];
  dofs[0] = imu.readWord(0x3B);
  dofs[1] = imu.readWord(0x3D);
  dofs[2] = imu.readWord(0x3F);
  dofs[3] = imu.readWord(0x43);
  dofs[4] = imu.readWord(0x45);
  dofs[5] = imu.readWord(0x47);
  uint8_t buf[32], buf2[32];
  buf[0] = 'I';
  buf[1] = 'M';
  buf[2] = 'U';
  memcpy(buf+3, dofs, 12);
  size_t buflen = COBSEncode((uint8_t *) buf, 15, buf2, 32);
  Serial.write(buf2, buflen);
}

byte spi_read(byte read_command) {
  digitalWrite(imuSelect, LOW);
  SPI.transfer(read_command);
  byte miso_data = SPI.transfer(0);
  digitalWrite(imuSelect, HIGH);
  return miso_data;
}
