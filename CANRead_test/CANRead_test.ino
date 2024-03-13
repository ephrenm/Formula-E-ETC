/*
  CANRead

  Receive and read CAN Bus messages

  See the full documentation here:
  https://docs.arduino.cc/tutorials/uno-r4-wifi/can
*/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_CAN.h>

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }

  if (!CAN.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {}
  }
}

static uint8_t Percentage, Pedal_flag, Brake_flag;

void loop()
{
  if (CAN.available())
  {
    CanMsg const msg = CAN.read();
    //Serial.println(msg.data[0]);
    Percentage= msg.data[1];
    Brake_flag=msg.data[0]& 1;     //first bit
    Pedal_flag=msg.data[0]>>1;     //second bit


    Serial.print("Throttle Percentage: ");
    Serial.print(Percentage);
    Serial.print("%  Pedal Flag: ");
    Serial.print(Pedal_flag);
    Serial.print("   Brake Flag: ");
    Serial.println(Brake_flag);
  }
}
