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
}

void loop() 
{
  int i=0;
  IBus_loop();

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
  delay(0.5);
  
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
