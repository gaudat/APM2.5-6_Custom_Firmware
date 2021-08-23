#include "MPU6000.h"

//Class constructor
MPU6000::MPU6000(){
}

void MPU6000::initialize(){

  //De-select all devices on SPI bus
  pinMode(imuSelect, OUTPUT);
  pinMode(pressureSelect, OUTPUT);
  digitalWrite(imuSelect, HIGH);
  digitalWrite(pressureSelect, HIGH);

  //Initialize SPI Bus
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); //Set to 1 Mhz
  SPI.setDataMode(SPI_MODE3);

  writeReg(0x6B, 0x80); // DEVICE_RESET
  delay(100);
  writeReg(0x6A, 0x07); // FIFO_RESET | I2C_MST_RESET | SIG_COND_RESET
  delay(100);
  writeReg(0x6B, 0x01); // CLKSEL = X axis gyroscope reference
  writeReg(0x38, 0x00); // Disable interrupt output
  writeReg(0x23, 0x00); // Disable FIFO
  writeReg(0x1C, 0x00); // Accel full scale = +/- 2g
  writeReg(0x37, 0x80); // INT level active low
  writeReg(0x19, 0x00); // Sample rate divider = 1
  writeReg(0x1A, 0x03); // DLPF: Accel 44 Hz, Gyro 42 Hz
  writeReg(0x1B, 0x18); // Gyro full scale = +/- 2000 deg/s
   gyroScale = getGyroScale();
   accelScale = getAccelScale();
}

bool MPU6000::testConnection(){
  uint8_t id;
  id = readReg(MPUREG_WHOAMI);

  if(id == 0x68){
    return true;
  }else{
    return false;
  }
}

void MPU6000::readScaled(){
  int16_t a[3], g[3];
  readImu(a, g);
  for(int j = 0; j<3; j++){
    accel[j] = a[j]*accelScale;
    gyro[j] = g[j]*gyroScale;
  }
  return;
}

void MPU6000::readImu(int16_t accel[3], int16_t gyro[3]){
  uint8_t byte_H, byte_L, dump;

  //Start a stream read of the sensor outputs:
  uint8_t addr = MPUREG_ACCEL_XOUT_H | 0x80;

  digitalWrite(imuSelect, LOW);

  dump = SPI.transfer(addr);

  // Read AccelX
  byte_H = SPI.transfer(0);
  byte_L = SPI.transfer(0);
  accel[0] = ((int)byte_H << 8) | byte_L;

  // Read AccelY
  byte_H = SPI.transfer(0);
  byte_L = SPI.transfer(0);
  accel[1] = ((int)byte_H << 8) | byte_L;

  // Read AccelZ
  byte_H = SPI.transfer(0);
  byte_L = SPI.transfer(0);
  accel[2] = ((int)byte_H << 8)| byte_L;

  // Read Temp
  byte_H = SPI.transfer(0);
  byte_L = SPI.transfer(0);
  mpu_temp = ((int)byte_H << 8)| byte_L;

  // Read GyroX
  byte_H = SPI.transfer(0);
  byte_L = SPI.transfer(0);
  gyro[0] = ((int)byte_H << 8)| byte_L;

  // Read GyroY
  byte_H = SPI.transfer(0);
  byte_L = SPI.transfer(0);
  gyro[1] = ((int)byte_H << 8) | byte_L;

  // Read GyroZ
  byte_H = SPI.transfer(0);
  byte_L = SPI.transfer(0);
  gyro[2] = ((int)byte_H <<8 ) | byte_L;

  digitalWrite(imuSelect, HIGH);
}

float MPU6000::getAccelScale(){
  const int bitNum = 32768;//(2^16)/2 Internal 16bit ADC
  uint8_t configByte;
  configByte = readReg(MPUREG_ACCEL_CONFIG);
  //Eliminate all bits except scale bits
  configByte &= 0x18;
  configByte = configByte >> 3;
  Serial.println(configByte);
  if (configByte == 0){
    return(abs((float)2.0/bitNum));
  }
  else if (configByte == 1){
    return(abs((float)4.0/bitNum));
  }
  else if (configByte == 2){
    return(abs((float)8.0/bitNum));
  }
  else if (configByte == 3){
    return(abs((float)16.0/bitNum));
  }
  else{
    return ((float)-1.0);
  }
}

float MPU6000::getGyroScale(){
  const int bitNum = 32768;//(2^16)/2 Internal 16bit ADC
  uint8_t configByte;
  configByte = readReg(MPUREG_GYRO_CONFIG);
  //Eliminate all bits except scale bits
  configByte &= 0x18;
  configByte = configByte >> 3;
  Serial.println(configByte);
  if (configByte == 0){
    return(abs((float)250.0/bitNum));
  }
  else if (configByte == 1){
    return(abs((float)500.0/bitNum));
  }
  else if (configByte == 2){
    return(abs((float)1000.0/bitNum));
  }
  else if (configByte == 3){
    return(abs((float)2000.0/bitNum));
  }
  else{
    return ((float)-1.0);
  }
}

uint8_t MPU6000::readReg(uint8_t reg){
  uint8_t value;
  digitalWrite(imuSelect, LOW);
  SPI.transfer(reg | 0x80); //Set read bit
  value = SPI.transfer((uint8_t) 0x00);
  digitalWrite(imuSelect, HIGH);
  return value;
}

void MPU6000::writeReg(uint8_t reg, uint8_t data){
  uint8_t dump;

  digitalWrite(imuSelect, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(imuSelect, HIGH);
}

// Modified from jrowberg/i2cdevlib /Arduino/MPU6050/MPU6050.cpp

int16_t MPU6000::readWord(uint8_t reg) {
  uint8_t high, low;
  digitalWrite(imuSelect, LOW);
  SPI.transfer(reg | 0x80);
  high = SPI.transfer(0);
  low = SPI.transfer(0);
  digitalWrite(imuSelect, HIGH);
  int16_t out = (high << 8) | low;
  return out;
}

void MPU6000::writeWord(uint8_t reg, uint8_t high, uint8_t low) {
  digitalWrite(imuSelect, LOW);
  SPI.transfer(reg);
  SPI.transfer(high);
  SPI.transfer(low);
  digitalWrite(imuSelect, HIGH);
}

void MPU6000::CalibrateGyro(uint8_t Loops ) {
  double kP = 0.3;
  double kI = 90;
  float x;
  x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
  kP *= x;
  kI *= x;
  
  PID( 0x43,  kP, kI, Loops);
}

void MPU6000::CalibrateAccel(uint8_t Loops ) {

	float kP = 0.3;
	float kI = 20;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;
	PID( 0x3B, kP, kI, Loops);
}

void MPU6000::PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops){
	uint8_t SaveAddress = (ReadAddress == 0x3B)?(0x06):0x13;

	int16_t  Data;
	float Reading;
	int16_t BitZero[3];
	uint8_t shift =(SaveAddress == 0x77)?3:2;
	float Error, PTerm, ITerm[3];
	int16_t eSample;
	uint32_t eSum ;
	Serial.write('>');
	for (int i = 0; i < 3; i++) {
		Data = readWord(SaveAddress + (i * shift)); // reads 1 or more 16 bit integers (Word)
		Reading = Data;
		if(SaveAddress != 0x13){
			BitZero[i] = Data & 1;										 // Capture Bit Zero to properly handle Accelerometer calibration
			ITerm[i] = ((float)Reading) * 8;
			} else {
			ITerm[i] = Reading * 4;
		}
	}
	for (int L = 0; L < Loops; L++) {
		eSample = 0;
		for (int c = 0; c < 100; c++) {// 100 PI Calculations
			eSum = 0;
			for (int i = 0; i < 3; i++) {
				Data = readWord(ReadAddress + (i * 2)); // reads 1 or more 16 bit integers (Word)
				Reading = Data;
				if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= 16384;	//remove Gravity
				Error = -Reading;
				eSum += abs(Reading);
				PTerm = kP * Error;
				ITerm[i] += (Error * 0.001) * kI;				// Integral term 1000 Calculations a second = 0.001
				if(SaveAddress != 0x13){
					Data = round((PTerm + ITerm[i] ) / 8);		//Compute PID Output
					Data = ((Data)&0xFFFE) |BitZero[i];			// Insert Bit0 Saved at beginning
				} else Data = round((PTerm + ITerm[i] ) / 4);	//Compute PID Output
        writeWord(SaveAddress + (i * shift), (Data >> 8) & 0xff, Data & 0xff);
			}
			if((c == 99) && eSum > 1000){						// Error is still to great to continue 
				c = 0;
				Serial.write('*');
			}
			if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;	// Successfully found offsets prepare to  advance
			if((eSum < 100) && (c > 10) && (eSample >= 10)) break;		// Advance to next Loop
			delay(1);
		}
		Serial.write('.');
		kP *= .75;
		kI *= .75;
		for (int i = 0; i < 3; i++){
			if(SaveAddress != 0x13) {
				Data = round((ITerm[i] ) / 8);		//Compute PID Output
				Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
			} else Data = round((ITerm[i]) / 4);
      writeWord(SaveAddress + (i * shift), (Data >> 8) & 0xff, Data & 0xff);
		}
	}
  writeReg(0x6A, 0x0C);
}

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) { Serial.print(F(Name)); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(F(EndTxt)); }//Name,Variable,Spaces,Precision,EndTxt
void MPU6000::PrintActiveOffsets() {
	uint8_t AOffsetRegister = 0x06;
	int16_t Data[3];
	//Serial.print(F("Offset Register 0x"));
	//Serial.print(AOffsetRegister>>4,HEX);Serial.print(AOffsetRegister&0x0F,HEX);
	Serial.print(F("\n//           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n//OFFSETS   "));
	if(AOffsetRegister == 0x06)	{
    Data[0] = readWord(AOffsetRegister);
    Data[1] = readWord(AOffsetRegister + 2);
    Data[2] = readWord(AOffsetRegister + 4);
  }
	//	A_OFFSET_H_READ_A_OFFS(Data);
	printfloatx("", Data[0], 5, 0, ",  ");
	printfloatx("", Data[1], 5, 0, ",  ");
	printfloatx("", Data[2], 5, 0, ",  ");
  Data[0] = readWord(0x13);
  Data[1] = readWord(0x15);
  Data[2] = readWord(0x17);
	//	XG_OFFSET_H_READ_OFFS_USR(Data);
	printfloatx("", Data[0], 5, 0, ",  ");
	printfloatx("", Data[1], 5, 0, ",  ");
	printfloatx("", Data[2], 5, 0, "\n");
}


bool MPU6000::getDMPEnabled() {
  uint8_t res = readReg(0x6A);
  return res & 0x80;
}
void MPU6000::setDMPEnabled(bool enabled) {
  writeReg(0x6A, enabled ? 0xCC : 0x0C);
}
