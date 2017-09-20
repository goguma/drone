/*
 * Test FlySky IBus interface on an Arduino Uno By SoftwareSerial
 *  Connect FS-iA6B receiver to Serial1.
 *  By Seongwon Cho

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 */
#include <SoftwareSerial.h>
#include <Wire.h>

#define DEBUG_IBUS 0
#define DEBUG_ACC_GYRO 0
#define DEBUG_PROCESSING 0

typedef enum _State
{
    GET_LENGTH,
    GET_DATA,
    GET_CHKSUML,
    GET_CHKSUMH,
    DISCARD,
}State;

static const uint8_t PROTOCOL_LENGTH = 0x20;
static const uint8_t PROTOCOL_OVERHEAD = 3; // <len><cmd><data....><chkl><chkh>
static const uint8_t PROTOCOL_TIMEGAP = 3; // Packets are received very ~7ms so use ~half that for the gap
static const uint8_t PROTOCOL_CHANNELS = 10;
static const uint8_t PROTOCOL_COMMAND40 = 0x40; // Command is always 0x40

typedef struct _FlySkyIBus
{
  uint8_t state;
  uint32_t last;
  uint8_t buffer[PROTOCOL_LENGTH];
  uint8_t ptr;
  uint8_t len;
  uint16_t channel[PROTOCOL_CHANNELS];
  uint16_t chksum;
  uint8_t lchksum;
}FlySkyIBus;

typedef struct _GY86
{
  const int MPU_addr = 0x68; // I2C address of the MPU-6050
  int16_t AcX;
  int16_t AcY;
  int16_t AcZ;
  int16_t Tmp;
  int16_t GyX;
  int16_t GyY;
  int16_t GyZ;
}GY86;

typedef struct _ACCEL_GYRO
{
  float dt;
  float Accl_Angle_X;
  float Accl_Angle_Y;
  float Accl_Angle_Z;
  float Gyro_Angle_X;
  float Gyro_Angle_Y;
  float Gyro_Angle_Z;
  float Filtered_Angle_X;
  float Filtered_Angle_Y;
  float Filtered_Angle_Z;
  float BaseAcX;
  float BaseAcY;
  float BaseAcZ;
  float BaseTmp;
  float BaseGyX;
  float BaseGyY;
  float BaseGyZ;
  float gyro_x;
  float gyro_y;
  float gyro_z;
}ACCEL_GYRO;

FlySkyIBus IBus;
SoftwareSerial mySerial(10, 11); // RX, TX
GY86 gy86_raw;
ACCEL_GYRO accel_gyro;
unsigned long t_now, t_prev;

void setup() 
{
  /*
   * begin Hardware Serial to display information to debug 
   */
  Serial.begin(115200);
  /*
   * begin SoftwareSerial and read data from IBus
   */
  IBus_begin();
  ////////////////////////////////////////////////////////
  /*
   * setup mpu6050
   */
  GY86_begin();
  GY86_Calibration();
  ////////////////////////////////////////////////////////
  /*
   * to get dt
   */
  startDT();
}

void loop() 
{
  IBus_loop();
  GY86_ReadData();
  endDT();
  GY86_ACCEL_YPR();
  GY86_GYRO_YPR();
  GY86_FILTERED_YPR();
  
#if DEBUG_PROCESSING
  static int cnt = 0;
  cnt++;
  if (cnt%2 == 0)
    SendDataToProcessing();

  if (cnt == 1000)
    cnt = 0;
#endif

#if DEBUG_ACC_GYRO
  print_accel_gyro_raw();
  delay(333);
#endif

}

static void print_ibus(void)
{
  Serial.print("CH[1] : ");
  Serial.print(IBus_readChannel(0), DEC);
  Serial.print(" | ");
  Serial.print("CH[2] : ");
  Serial.print(IBus_readChannel(1), DEC);
  Serial.print(" | ");
  Serial.print("CH[3] : ");
  Serial.print(IBus_readChannel(2), DEC);
  Serial.print(" | ");
  Serial.print("CH[4] : ");
  Serial.print(IBus_readChannel(3), DEC);
  Serial.print(" | ");
  Serial.print("CH[5] : ");
  Serial.print(IBus_readChannel(4), DEC);
  Serial.print(" | ");
  Serial.print("CH[6] : ");
  Serial.print(IBus_readChannel(5), DEC);
  Serial.print(" | ");
  Serial.print("CH[7] : ");
  Serial.print(IBus_readChannel(6), DEC);
  Serial.print(" | ");
  Serial.print("CH[8] : ");
  Serial.print(IBus_readChannel(7), DEC);
  Serial.print(" | ");
  Serial.print("CH[9] : ");
  Serial.print(IBus_readChannel(8), DEC);
  Serial.print(" | ");
  Serial.print("CH[10] : ");
  Serial.print(IBus_readChannel(9), DEC);
  Serial.println("");
}

static void print_accel_gyro_raw(void)
{
  Serial.print("AcX = "); Serial.print(gy86_raw.AcX);
  Serial.print(" | AcY = "); Serial.print(gy86_raw.AcY);
  Serial.print(" | AcZ = "); Serial.print(gy86_raw.AcZ);
  Serial.print(" | Tmp = "); Serial.print(gy86_raw.Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(gy86_raw.GyX);
  Serial.print(" | GyY = "); Serial.print(gy86_raw.GyY);
  Serial.print(" | GyZ = "); Serial.println(gy86_raw.GyZ);
}

static void SendDataToProcessing() {
  Serial.print(F("DEL:"));
  Serial.print(accel_gyro.dt, DEC);
  Serial.print(F("#ACC:"));
  Serial.print(accel_gyro.Accl_Angle_X, 2);
  Serial.print(F(","));
  Serial.print(accel_gyro.Accl_Angle_Y, 2);
  Serial.print(F(","));
  Serial.print(accel_gyro.Accl_Angle_Z, 2);
  Serial.print(F("#GYR:"));
  Serial.print(accel_gyro.Gyro_Angle_X, 2);
  Serial.print(F(","));
  Serial.print(accel_gyro.Gyro_Angle_Y, 2);
  Serial.print(F(","));
  Serial.print(accel_gyro.Gyro_Angle_Z, 2);
  Serial.print(F("#FIL:"));
  Serial.print(accel_gyro.Filtered_Angle_X, 2);
  Serial.print(F(","));
  Serial.print(accel_gyro.Filtered_Angle_Y, 2);
  Serial.print(F(","));
  Serial.print(accel_gyro.Filtered_Angle_Z, 2);
  Serial.println(F(""));
  delay(5);  
}

static void startDT()
{
  t_prev = micros();
}

static void endDT()
{
  t_now = micros();
  accel_gyro.dt = (t_now - t_prev) / 1000000.0;
  t_prev = t_now;
}



static void GY86_begin(void)
{
  /*
   * setup mpu6050
   */
   Wire.begin();
   Wire.beginTransmission(gy86_raw.MPU_addr);
   Wire.write(0x6B);// PWR_MGMT_1 register
   Wire.write(0);// set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);
   ////////////////////////////////////////////////////////
}

static void GY86_Calibration(void)
{
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  GY86_ReadData();
#define GY86_CALIBRATION_CNT 10

  for (int i = 0; i < GY86_CALIBRATION_CNT; i++)
  {
    GY86_ReadData();
    sumAcX += gy86_raw.AcX, sumAcY += gy86_raw.AcY, sumAcZ += gy86_raw.AcZ;
    sumGyX += gy86_raw.GyX, sumGyY += gy86_raw.GyY, sumGyZ += gy86_raw.GyZ;
    delay(100);
  }

  accel_gyro.BaseAcX = sumAcX / GY86_CALIBRATION_CNT;
  accel_gyro.BaseAcY = sumAcY / GY86_CALIBRATION_CNT;
  accel_gyro.BaseAcZ = sumAcZ / GY86_CALIBRATION_CNT;

  accel_gyro.BaseGyX = sumGyX / GY86_CALIBRATION_CNT;
  accel_gyro.BaseGyY = sumGyY / GY86_CALIBRATION_CNT;
  accel_gyro.BaseGyZ = sumGyZ / GY86_CALIBRATION_CNT;
}

static void GY86_ReadData(void)
{
  Wire.beginTransmission(gy86_raw.MPU_addr);
  Wire.write(0x3B); //starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(gy86_raw.MPU_addr,14,true); // request a total of 14 registers
  gy86_raw.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
  gy86_raw.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gy86_raw.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gy86_raw.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gy86_raw.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy86_raw.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gy86_raw.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

static void GY86_ACCEL_YPR(void)
{
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/3.14159;
  //const float RADIANS_TO_DEGREES = 180/M_PI;
  /*When I use M_PI instead 3.14159 GyZ was strange*/
  
  accel_x = gy86_raw.AcX - accel_gyro.BaseAcX;
  accel_y = gy86_raw.AcY - accel_gyro.BaseAcY;
  accel_z = gy86_raw.AcZ + (16384 - accel_gyro.BaseAcZ);

  accel_yz = sqrt(pow(accel_y,2) + pow(accel_z,2));
  accel_gyro.Accl_Angle_Y = atan(-accel_x/accel_yz)*RADIANS_TO_DEGREES;

  accel_xz = sqrt(pow(accel_x,2) + pow(accel_z,2));
  accel_gyro.Accl_Angle_X = atan( accel_y/accel_xz)*RADIANS_TO_DEGREES;
    
  accel_gyro.Accl_Angle_Z = 0;
}

static void GY86_GYRO_YPR(void)
{
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;
  
  accel_gyro.gyro_x = (gy86_raw.GyX - accel_gyro.BaseGyX)/GYROXYZ_TO_DEGREES_PER_SEC;
  accel_gyro.gyro_y = (gy86_raw.GyY - accel_gyro.BaseGyY)/GYROXYZ_TO_DEGREES_PER_SEC;
  accel_gyro.gyro_z = (gy86_raw.GyZ - accel_gyro.BaseGyZ)/GYROXYZ_TO_DEGREES_PER_SEC;
  
  /* below code doesn't be overflowed ??? */
  accel_gyro.Gyro_Angle_X += accel_gyro.gyro_x * accel_gyro.dt;
  accel_gyro.Gyro_Angle_Y += accel_gyro.gyro_y * accel_gyro.dt;
  accel_gyro.Gyro_Angle_Z += accel_gyro.gyro_z * accel_gyro.dt; 
}

static void GY86_FILTERED_YPR(void)
{
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = accel_gyro.Filtered_Angle_X + accel_gyro.gyro_x * accel_gyro.dt;
  tmp_angle_y = accel_gyro.Filtered_Angle_Y + accel_gyro.gyro_y * accel_gyro.dt;
  tmp_angle_z = accel_gyro.Filtered_Angle_Z + accel_gyro.gyro_z * accel_gyro.dt;
  
  accel_gyro.Filtered_Angle_X = 
    ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_gyro.Accl_Angle_X;    
  accel_gyro.Filtered_Angle_Y = 
    ALPHA * tmp_angle_y + (1.0-ALPHA) * accel_gyro.Accl_Angle_Y;    
  accel_gyro.Filtered_Angle_Z = tmp_angle_z;
}

static void IBus_begin(void)
{
  mySerial.begin(115200);

  IBus.state = DISCARD;
  IBus.last = millis();
  IBus.ptr = 0;
  IBus.len = 0;
  IBus.chksum = 0;
  IBus.lchksum = 0;
}

static void IBus_loop(void)
{
  while (mySerial.available() > 0)
  {
    uint32_t now = millis();
    if (now - IBus.last >= PROTOCOL_TIMEGAP)
    {
      IBus.state = GET_LENGTH;
    }
    IBus.last = now;
    
    uint8_t v = mySerial.read();
    switch (IBus.state)
    {
      case GET_LENGTH:
        if (v <= PROTOCOL_LENGTH)
        {
          IBus.ptr = 0;
          IBus.len = v - PROTOCOL_OVERHEAD;
          IBus.chksum = 0xFFFF - v;
          IBus.state = GET_DATA;
        }
        else
        {
          IBus.state = DISCARD;
        }
        break;

      case GET_DATA:
        IBus.buffer[IBus.ptr++] = v;
        IBus.chksum -= v;
        if (IBus.ptr == IBus.len)
        {
          IBus.state = GET_CHKSUML;
        }
        break;
        
      case GET_CHKSUML:
        IBus.lchksum = v;
        IBus.state = GET_CHKSUMH;
        break;

      case GET_CHKSUMH:
        // Validate checksum
        if (IBus.chksum == (v << 8) + IBus.lchksum)
        {
          // Execute command - we only know command 0x40
          switch (IBus.buffer[0])
          {
            case PROTOCOL_COMMAND40:
              // Valid - extract channel data
              for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2)
              {
                IBus.channel[i / 2] = IBus.buffer[i] | (IBus.buffer[i + 1] << 8);
              }
              break;

            default:
              break;
          }
        }
        IBus.state = DISCARD;
        break;

      case DISCARD:
      default:
        break;
    }
  }
}

static uint16_t IBus_readChannel(uint8_t channelNr)
{
  if (channelNr < PROTOCOL_CHANNELS)
  {
    return IBus.channel[channelNr];
  }
  else
  {
    return 0;
  }
}
