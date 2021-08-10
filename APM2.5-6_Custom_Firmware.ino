#include <SPI.h>
#include "APM_PPM.h"
#include "APM2RCOutput.h"

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

class SlewRateLimiter {
  public:
  SlewRateLimiter(int limit_per_tick, bool limit_accel_only) : 
  neutral(0),
  first_run(false),
  limit_per_tick(limit_per_tick),
  limit_accel_only(limit_accel_only)
   {

  }
  int run(int input, long ticks) {
    if (first_run) {
      last_ticks = ticks;
      last_output = input;
      first_run = false;
      return input;
    }
    long time_diff = ticks - last_ticks;
    last_ticks = ticks;
    // See if accel or decel
    if (last_output >= neutral && input > last_output) {
      // Forward Accel 
      int upper_limit = last_output + time_diff * limit_per_tick;
      input = constrain(input, neutral, upper_limit);
    }
    else if (last_output >= neutral && input <= last_output) {
      // Forward Decel
      if (!limit_accel_only) {
        int lower_limit = last_output - time_diff * limit_per_tick;
        // Prevent output from going under neutral
        if (lower_limit < neutral) {
          lower_limit = neutral;
        }
        input = constrain(input, lower_limit, last_output);
      }
    }
    else if (last_output < neutral && input < last_output) {
      // Backward Accel
      int lower_limit = last_output - time_diff * limit_per_tick;
      input = constrain(input, lower_limit, neutral);
    }
    else if (last_output < neutral && input >= last_output) {
      // Backward Decel
      if (!limit_accel_only) {
        int upper_limit = last_output + time_diff * limit_per_tick;
        // Prevent output from going above neutral
        if (upper_limit > neutral) {
          upper_limit = neutral;
        }
        input = constrain(input, last_output, upper_limit);
      }
    }
    last_output = input;
    return last_output;
  }
  private:
  int neutral;
  long last_ticks;
  int last_output;
  bool first_run;
  int limit_per_tick;
  bool limit_accel_only;
};

SlewRateLimiter throttle_SLM(1, true);
APM2RCOutput output;

void motor_loop() {
  // Front = +y (Ch 3), Right = +x (Ch 1)
  const int front_ch = 2;
  const int right_ch = 0;
  
  const int neutral = 1500;
  const int input_deadzone = 50;

  uint16_t inputs[numChannel];
  const long input_ts = millis();

  rxin.read(inputs);

  // Add input channel preproc here

  int throttle = inputs[front_ch];
  int steering = inputs[right_ch];

  throttle -= neutral;
  steering -= neutral;

  // Add throttle steering proc here
  throttle = throttle_SLM.run(throttle, input_ts);

  // Throttle -> Motor left +1, Motor right -1
  // Steering -> Motor left +1, Motor right +1
  // Motor left = neutral + Throttle + Steering
  // Motor right = neutral - Throttle + Steering
  int motor_left = neutral - throttle + steering;
  int motor_right = neutral + throttle + steering;

  // Add motor output proc here
  motor_left = constrain(motor_left, 1300, 1700);
  motor_right = constrain(motor_right, 1300, 1700);

  bool oe = inputs[5] > 1200;
  // Write to ouput
  if (oe) {

  output.write(0, motor_left);
  output.write(1, motor_right);
  } else {
        
  output.write(0, neutral);
  output.write(1, neutral);
  }
}

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
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE3);
  delay(100);

  output.init();
  for (int i = 0; i < 8; i++) {
  output.enable_ch(i);
  }

  rxin.initialize();
}

void servos_mirror() {
  uint16_t outputs[8];
  
  rxin.read(outputs);
  output.write(0, outputs, 8);
}

void loop() {
  digitalWrite(ledRed, !digitalRead(ledRed));
  motor_loop();
}

void loop_ppm_pass() {
  digitalWrite(ledBlue, digitalRead(ppmIn));
}

void loop_ppm_print () {
  Serial.println(rxin.newData());
  uint16_t val[numChannel];
  rxin.read(val);
  for (int ch=0; ch < numChannel; ch++) {
    Serial.print(val[ch]);
    Serial.print(" ");
  }
  Serial.println();
  delay(1000);
}

void loop_spi_print () {
  
  byte resultat = spi_read(0x3B | 0b10000000);
  Serial.println(resultat, BIN);
  delay(1000);

}

byte spi_read(byte read_command) {
  digitalWrite(imuSelect, LOW);
  SPI.transfer(read_command);
  byte miso_data = SPI.transfer(0);
  digitalWrite(imuSelect, HIGH);
  return miso_data;
}
