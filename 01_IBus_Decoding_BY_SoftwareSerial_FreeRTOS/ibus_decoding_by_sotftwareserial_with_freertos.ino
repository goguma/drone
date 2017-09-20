#include <Arduino_FreeRTOS.h>
#include <croutine.h>
#include <event_groups.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOSVariant.h>
#include <list.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <StackMacros.h>
#include <task.h>
#include <timers.h>

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
//#include <inttypes.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define DEBUG_SERIAL_THREAD 0
#define DEBUG_IBUS 1
#define DEBUG_ACC_GYRO 1
//For FlySkyIBus
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

FlySkyIBus IBus;
SoftwareSerial mySerial(10, 11); // RX, TX
////////////////////////////////////////////////////////

//For MPU6050
const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
////////////////////////////////////////////////////////

static void Thread_RF_Ibus(void *arg)
{
  while(1)
  {
    IBus_loop();

#if DEBUG_IBUS
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
#endif

    //vTaskDelay((100L * configTICK_RATE_HZ) / 1000L); //100ms delay
    delay(333);
  }
}

static void Thread_GY_86(void *arg)
{
  while(1)
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); //starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

#if DEBUG_ACC_GYRO
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
#endif

    //vTaskDelay((333L * configTICK_RATE_HZ) / 1000L); //100ms delay
    delay(333);
  }
}

static void Thread_Debug(void *arg)
{
  while(1)
  {
#if DEBUG_SERIAL_THREAD
#if DEBUG_IBUS
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
#endif
#if DEBUG_ACC_GYRO
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
    Serial.print(" | GyX = "); Serial.print(GyX);
    Serial.print(" | GyY = "); Serial.print(GyY);
    Serial.print(" | GyZ = "); Serial.println(GyZ);
#endif
#endif
    vTaskDelay((500L * configTICK_RATE_HZ) / 1000L); //500ms delay
  }
}

void setup() 
{
  portBASE_TYPE s1, s2, s3;

  /*
   * begin SoftwareSerial and read data from IBus
   */
  IBus_begin();
  ////////////////////////////////////////////////////////
  /*
   * setup mpu6050
   */
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);// PWR_MGMT_1 register
  Wire.write(0);// set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  ////////////////////////////////////////////////////////

  /*
   * begin Hardware Serial to display information to debug 
   */
  Serial.begin(115200);
  
  s1 = xTaskCreate(Thread_RF_Ibus, "RF_IBUS", configMINIMAL_STACK_SIZE, NULL, 1, NULL); //pririty 1 to test
  if (s1 != pdPASS)
  {
    Serial.println(F("Failed to create thread of Thread_RF_Ibus"));
    while(1);
  }

  s2 = xTaskCreate(Thread_GY_86, "ACCL_GYRO", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL); //priority 1 to test
  if (s2 != pdPASS)
  {
    Serial.println(F("Failed to create thread of Thread_GY_86"));
    while(1);
  }

  //s3 = xTaskCreate(Thread_Debug, "DEBUG", configMINIMAL_STACK_SIZE, NULL, 1, NULL); //priority 1 to test
  //if (s3 != pdPASS)
  //{
  //  Serial.println(F("Failed to create thread of Thread_Display_TO_DEBUG"));
  //  while(1);
  //}

  vTaskStartScheduler();
  Serial.println(F("Insufficient RAM"));

  while(1);

}

void loop() 
{  
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

void IBus_loop(void)
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

uint16_t IBus_readChannel(uint8_t channelNr)
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
