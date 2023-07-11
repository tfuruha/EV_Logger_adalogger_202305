#include "delay.h"
//refer to https://tekuteku-embedded.xyz/2022/09/15/adxl345/
//File Name:tinyADXL345.h
//architecture: ESP32
//I2Cを使う

#ifndef TwoWire_h
#include <Wire.h>
#endif

//device address
#define ADXL345_I2CADDR_DEFAULT (uint8_t)(0x53)  //SDO LO: 0x53, SDO HI: 0x1D
 
//Register list
#define ADXL345_DEVID         (0x00) //< Device ID
#define ADXL345_THRESH_TAP    (0x1D) //< Tap threshold
#define ADXL345_OFSX          (0x1E) //< X-axis offset
#define ADXL345_OFSY          (0x1F) //< Y-axis offset
#define ADXL345_OFSZ          (0x20) //< Z-axis offset
#define ADXL345_DUR           (0x21) //< Tap duration
#define ADXL345_LATENT        (0x22) //< Tap latency
#define ADXL345_WINDOW        (0x23) //< Tap window
#define ADXL345_THRESH_ACT    (0x24) //< Activity threshold
#define ADXL345_THRESH_INACT  (0x25) //< Inactivity threshold
#define ADXL345_TIME_INACT    (0x26) //< Inactivity time
#define ADXL345_ACT_INACT_CTL (0x27) //< Axis enable control for activity and inactivity detection
#define ADXL345_THRESH_FF     (0x28) //< Free-fall threshold
#define ADXL345_TIME_FF       (0x29) //< Free-fall time
#define ADXL345_TAP_AXES      (0x2A) //< Axis control for single/double tap
#define ADXL345_ACT_TAP_STATUS (0x2B) //< Source for single/double tap
#define ADXL345_BW_RATE       (0x2C) //< Data rate and power mode control
#define ADXL345_POWER_CTL     (0x2D) //< Power-saving features control
#define ADXL345_INT_ENABLE    (0x2E) //< Interrupt enable control
#define ADXL345_INT_MAP       (0x2F) //< Interrupt mapping control
#define ADXL345_INT_SOURCE    (0x30) //< Source of interrupts
#define ADXL345_DATA_FORMAT   (0x31) //< Data format control
#define ADXL345_DATAX0        (0x32) //< X-axis data 0
#define ADXL345_DATAX1        (0x33) //< X-axis data 1
#define ADXL345_DATAY0        (0x34) //< Y-axis data 0
#define ADXL345_DATAY1        (0x35) //< Y-axis data 1
#define ADXL345_DATAZ0        (0x36) //< Z-axis data 0
#define ADXL345_DATAZ1        (0x37) //< Z-axis data 1
#define ADXL345_FIFO_CTL      (0x38) //< FIFO control
#define ADXL345_FIFO_STATUS   (0x39) //< FIFO status
/*=========================================================================*/

//Range Bits
typedef enum {
  Range2g   = 0b00,
  Range4g   = 0b01,
  Range8g   = 0b10,
  Range16g  = 0b11
} adxl345_range_t;

//Output Data Rate Bits
typedef enum {
  DATARATE_3200_HZ = 0b1111, //F < 1600Hz Bandwidth   140uA IDD
  DATARATE_1600_HZ = 0b1110, //E <  800Hz Bandwidth    90uA IDD
  DATARATE_800_HZ  = 0b1101, //D <  400Hz Bandwidth   140uA IDD
  DATARATE_400_HZ  = 0b1100, //C <  200Hz Bandwidth   140uA IDD
  DATARATE_200_HZ  = 0b1011, //B <  100Hz Bandwidth   140uA IDD
  DATARATE_100_HZ  = 0b1010, //A <   50Hz Bandwidth   140uA IDD
  DATARATE_50_HZ   = 0b1001, //9 <   25Hz Bandwidth    90uA IDD
  DATARATE_25_HZ   = 0b1000, //8 < 12.5Hz Bandwidth    60uA IDD
  DATARATE_12_5_HZ = 0b0111, //7 < 6.25Hz Bandwidth    50uA IDD
  DATARATE_6_25HZ  = 0b0110, //6 < 3.13Hz Bandwidth    45uA IDD
  DATARATE_3_13_HZ = 0b0101, //5 < 1.56Hz Bandwidth    40uA IDD
  DATARATE_1_56_HZ = 0b0100, //4 < 0.78Hz Bandwidth    34uA IDD
  DATARATE_0_78_HZ = 0b0011, //3 < 0.39Hz Bandwidth    23uA IDD
  DATARATE_0_39_HZ = 0b0010, //2 < 0.20Hz Bandwidth    23uA IDD
  DATARATE_0_20_HZ = 0b0001, //1 < 0.10Hz Bandwidth    23uA IDD
  DATARATE_0_10_HZ = 0b0000  //0 < 0.05Hz Bandwidth    23uA IDD (default value)
} adxl345_dataRate_t;

float kgain = 0.004;  //Range16g
//float kgain = 0.016;  //Range4g

//指定のアドレスに値を書き込む関数
void WriteReg(uint8_t addrs, uint8_t val)
{
  Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);
  Wire.write(addrs);
  Wire.write(val);
  Wire.endTransmission(); 
}

//指定のアドレスに値を書き込む関数
void WriteRegN(uint8_t addrs, int8_t val)
{
  Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);
  Wire.write(addrs);
  Wire.write(val);
  Wire.endTransmission(); 
}

//指定のアドレスから値を読む関数(lengthで読み込むバイト数を指定する)
void ReadReg(uint8_t addrs,uint8_t * buf,size_t length)
{
  Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);
  Wire.write(addrs);
  Wire.endTransmission();
  Wire.requestFrom(ADXL345_I2CADDR_DEFAULT,length,true);
 
  for(uint8_t i =0;i<length;i++)
  {
  buf[i] = Wire.read();
  }
}
//指定のアドレスから値を読む関数(1バイト)
uint8_t ReadRegByte(uint8_t addrs){
  uint8_t var;
  ReadReg(addrs,&var,1);
  return var;
}

//2の補数計算(マイナス値を算出する）
int16_t two_complement(uint16_t val,uint8_t bits)
{
  int16_t val_out;
   
  if ((val) & ((uint16_t)1 << (bits - 1))) {
    val_out = val - (1U<<bits);
  }
  else{
    val_out = val;
  }
  return val_out;
}

//x,y,z軸の加速度を読み込む
void readAcc(float* x,float* y,float* z){
  uint8_t buf[6];
  ReadReg(ADXL345_DATAX0,buf,6);
  //符号ビットを考慮
  int16_t temp = two_complement((buf[1]<<8U | buf[0])&0x1FFF,13);
  //物理値変換
  *x=temp*kgain;
  temp = two_complement((buf[3]<<8U | buf[2])&0x1FFF,13);
  *y=temp*kgain;
  temp = two_complement((buf[5]<<8U | buf[4])&0x1FFF,13);
  *z=temp*kgain; 

}
//ADXL345との通信初期化
bool initADXL345(){
  Wire.begin();
  Wire.flush();
  //Is ADXL345 avail?
  int retVar;
  for(int i=0;i<5;i++){
    //初期通信はWireライブラリを使用する
    Wire.beginTransmission(ADXL345_I2CADDR_DEFAULT);  // Transmit to device
    Wire.write(ADXL345_DEVID);                        // Sends value byte
    retVar = Wire.endTransmission();                  // Stop transmitting
    if(retVar == 0){ //if success
      Wire.requestFrom((uint16_t)ADXL345_I2CADDR_DEFAULT,(size_t)1,true);  // Request from slave device
      retVar = Wire.read();         // Receive a byte as character
      Serial.print("DEVICE ID: 0x");  //for debug
      Serial.println(retVar,HEX);
      if(retVar==0xE5){
        return true;
      }
    }else{  //
      Serial.print("request error:");
      Serial.println(retVar,HEX);
    }
    delay(100);
  }
  Serial.println("ADXL345 is not avail.");
  return false;
} 

void ResetOffsetRegADXL345(){
  //Weite to OFSTx REGISTER
  WriteRegN(ADXL345_OFSX,0x00)  ;
  WriteRegN(ADXL345_OFSY,0x00)  ;
  WriteRegN(ADXL345_OFSZ,0x00)  ;  
}

void SetOffsetRegADXL345(){
  int NumOfSmp = 128;
  int AccX,AccY,AccZ;
  int SumX = 0; float SumY = 0; float SumZ = 0;
  uint8_t buf[6];
  //Place Seonsor in X=0g, Y=0g, Z=1g
  //Vs=ON, Vddio = ON
  delayMicroseconds(1100); //WAIT 1.1ms
  ResetOffsetRegADXL345();
  WriteReg(ADXL345_BW_RATE,0x0A);     //power mode DateRate 200Hz
  WriteReg(ADXL345_DATA_FORMAT,0x0B);  //16g 13bit mode
  WriteReg(ADXL345_POWER_CTL,0x08);  //START Measurement
  WriteReg(ADXL345_INT_ENABLE,0x80);  //ENABLE 
  delay(20);  //wait 11.1ms
  ReadReg(ADXL345_DATAX0,buf,6);
  //take NumOfSmp data points
  unsigned long tStart=micros(); 
  for(int i=0;i<NumOfSmp;i++){
    //Data Ready?
    while(true){
      uint8_t var = (ReadRegByte(ADXL345_INT_SOURCE)) & 0x80 ;
      if(var) break;
    }
    ReadReg(ADXL345_DATAX0,buf,6);
    AccX = two_complement((buf[1]<<8U | buf[0])&0x1FFF,13);
    AccY = two_complement((buf[3]<<8U | buf[2])&0x1FFF,13);
    AccZ = two_complement((buf[5]<<8U | buf[4])&0x1FFF,13);
    SumX = SumX + AccX; SumY = SumY + AccY; SumZ = SumZ + AccZ;
    delay(1);
    //for debug
    /* ***
    Serial.print(AccX);  Serial.print(",");  
    Serial.print(AccY);  Serial.print(",");  
    Serial.print(AccZ);  Serial.println("");  
     *** */
  }
  unsigned long tEnd=micros(); 
  Serial.print(tStart);  Serial.print(","); 
  Serial.print(tEnd);  Serial.println(","); 
  //and Average
  float AveX_0 = (float)SumX / (float)NumOfSmp;
  float AveY_0 = (float)SumY / (float)NumOfSmp;
  float AveZ_0 = (float)SumZ / (float)NumOfSmp;
  //calcuralte  Calibration value
  int8_t X_CAL=round(-(AveX_0/4.));
  int8_t Y_CAL=round(-(AveY_0/4.));
  int8_t Z_CAL=round(-1-((AveZ_0-255.)/4.));
  //int8_t Z_CAL=-((AveZ_0)/4);
  //for debug
  /* *** */
  Serial.print(X_CAL);  Serial.print(",");  
  Serial.print(Y_CAL);  Serial.print(",");  
  Serial.print(Z_CAL);  Serial.println("");  
  /* *** */
  //Weite to OFSTx REGISTER
  WriteRegN(ADXL345_OFSX,X_CAL)  ;
  WriteRegN(ADXL345_OFSY,Y_CAL)  ;
  WriteRegN(ADXL345_OFSZ,Z_CAL)  ;  

}
