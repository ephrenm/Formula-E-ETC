#include <Arduino_CAN.h>


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

static uint8_t accPercentage, accFlag, brakeFlag, brakePercentage;

void loop()
{
  if (CAN.available())
  {
    CanMsg const msg = CAN.read();

    brakePercentage = msg.data[0];
    brakeFlag = msg.data[1]; 
    accPercentage = msg.data[2];
    accFlag = msg.data[3];

    Serial.print("Throttle Percentage: ");
    Serial.print(accPercentage);
    Serial.print("%  Pedal Flag: ");
    Serial.print(accFlag);

    Serial.print("Brake Percentage: ");
    Serial.print(brakePercentage);
    Serial.print("   Brake Flag: ");
    Serial.println(Brake_flag);
  }
}
