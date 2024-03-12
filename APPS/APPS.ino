// [TODO]
// Add check for enable signal from master controller before outputting to I2C
// Add code to set a digital pin high whenever brake pedal is pressed (brake light activation for master controller)
// Out-of-range values from APPS to Arduino & between Arduino and motor controller - T.4.2.10 & T.4.3.4
// Test calibration in assembly
// Add check to calibration numbers before writing to EEPROM
// Add check to only allow calibration while accel < 10 (creates issues if throttle stuck > 10, maybe only while moving? or allow when accel > 10 and break engaged?)
// Improve Readibility: Add more comments, but variable names
// Need to make sure that upon calibration exit, output is 0, so that the car doesn't unexpectedly take off 
// Something has to RESET implausibility variables


// #include <EEPROM.h> // EEPROM LIBRARY
// #include <Adafruit_MCP4725.h> // I2C DAC LIBRARY

#include <Arduino_CAN.h>

#define analogPin5g A0  // 5V Gas Input     ONLY INPUT

// #define analogPin3g A7  // 3.3V Gas Input
// #define analogWritePin A0

// #define calibration_interrupt_pin 2 // Input Button, toggles the calibration state


#define brake_state_pin D12
#define implausibility_led D6 // Output LED, indicates if implausibility is detected
#define throttle_led D7 // Output LED, indicates if we outputting an accel value above

// #define RTD_pin 5

bool brakes_engaged = true;

// Variables for storing percentages mapped from analog readings
uint8_t accel_pedal_travel; // was gasperc, combined accel percentages/2 for final pedal travel percentage
uint8_t brake_pedal_travel; // was brakeperc, same as above

// Defining variables & initializing them as zero for potentiometer readings and calculated percentages
int analog_in_5acc = 0;
// int analog_in_3acc = 0;

uint8_t percent_5g = 0;
uint8_t percent_3g = 0;

bool pedal_implausibility = false; // pedal_implausibility is the state that occurs when the position of two matching pedals do not agree
bool brake_implausibility = false; // brake_implausibility is the state that occurs when brake is engaged and accel > 25%
unsigned long implausibility_timer = 0;

// variable init for pedal calibration mins/maxs
int accel_5v_max = 1000;
int accel_5v_min = 500;
// int accel_3v_max = 1000;
// int accel_3v_min = 300;

uint8_t pedal_implausibility_thres = 10; // a percentage of the max allowable 3v to 5v accel sensor shift
uint8_t pedal_deadzone_thres = 5; // a percentage of throttle to ignore (0 to [this value])

unsigned long debounce_millis = 0;

// Adafruit_MCP4725 dac; // create I2C DAC object

//CAN ADDRESS
static uint32_t const CAN_ID = 0x20;


// setup()
// Before we enter the loop() below, this setup() function is run once
void setup() {
  Serial.begin(115200);

  // dac.begin(0x62); // initialize I2C DAC at Addr: 0x62

  pinMode(analogPin5g, INPUT);
  // pinMode(analogPin3g, INPUT);
  // pinMode(gas_pin, OUTPUT);

  // pinMode(calibration_interrupt_pin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(calibration_interrupt_pin), calibration_reset, FALLING);

  pinMode(brake_state_pin, INPUT_PULLUP);
  // pinMode(RTD_pin, INPUT);

  //LED FLAGS
  pinMode(throttle_led, OUTPUT);
  pinMode(implausibility_led, OUTPUT);

  // pinMode(analogWritePin, OUTPUT);

  //CAN communication
  if (!CAN.begin(CanBitRate::BR_250k))
    {
      Serial.println("CAN.begin(...) failed.");
      for (;;) {}
    }



  analogWriteResolution(12); // unecessary for I2C DAC
  analogReadResolution(12);
}


static uint32_t msg_cnt = 0;

// loop()
// Main driver code, as long as the Arduino is on this code loops forever
void loop() {
  // Write LED statuses
  digitalWrite(throttle_led, accel_pedal_travel >= pedal_deadzone_thres);
  digitalWrite(implausibility_led, pedal_implausibility || brake_implausibility);

  // Read/update accel pedal travel, sets implausibility flag
  // read_accel_pedal_travel();

  //TODO something has to RESET implausibility FLAGS
  brake_implausibility=0;


  // Read/update brake pedal state, sets implausibility flag
  read_brake_pedal_travel();

  ////// CAN WRITE
  uint8_t can_out[] = {0,accel_pedal_travel}; 
  can_out[0]= brake_implausibility | pedal_implausibility<<1;

  // //CAN message object
  CanMsg const msg(CanStandardId(CAN_ID), sizeof(can_out), can_out);
  CAN.write(msg);

  //faulty check
  // if (int const rc = CAN.write(msg); rc < 0)
  // {
  //   Serial.print  ("CAN.write(...) failed with error code ");
  //   Serial.println(rc);
  //   for (;;) { }
  // }
  /////   END CAN

  // Write accel value if conditions are met, otherwise set output to 0
  // if (
  //   !pedal_implausibility // NO pedal implausibility
  //   && !brake_implausibility // NO break implausibility
  //   && accel_pedal_travel < pedal_deadzone_thres // Pedal travel above deadzone
  //   && digitalRead(RTD_pin) // Ready-to-drive signal is true
  // ) {
  //   write_accel_value_i2c();
  // } else {
  //   dac.setVoltage(0, false);
  // }

  // Dump state to serial
  print_state();
}

// read brake state, set implausibility flag
void read_brake_pedal_travel() {
  // Read brake state
  brakes_engaged = !digitalRead(brake_state_pin);

  //JUST TESTING
    analog_in_5acc = analogRead(analogPin5g); // 5V Gas INPUT

    //setting 5v accelerator boundaries
    if (analog_in_5acc > accel_5v_max) accel_5v_max = analog_in_5acc;
    if (analog_in_5acc < accel_5v_min) accel_5v_min = analog_in_5acc;
    //5V pedal as a percentage
    percent_5g = constrain(map(analog_in_5acc, accel_5v_min, accel_5v_max, 0, 100), 0, 100);
    accel_pedal_travel = percent_5g;

  // Brake implausibility check
  if (brakes_engaged && accel_pedal_travel > 25) {
      brake_implausibility = true;
      Serial.print("Implausability detected");
      delay(5000);
    // TODO: what happens if pedal travel is less than 25? condition falls out
  } else if (accel_pedal_travel < 6) {
    brake_implausibility = false;
  }
}

// write_accel_value_i2c()
// double check constrain values; replace map with a slope function and test difference in speed; 
// add check to see if setVoltage function returns false
// void write_accel_value_i2c() {
//   // Map accel values
//   uint8_t output = constrain(map(accel_pedal_travel, 0, 100, 820, 3280), 820, 3280);

//   // Write accel output
//   dac.setVoltage((uint16_t) output, false);
// }


// // write_accel_value()
// // transform pedal travel % to usable DAC 12-bit value
// void write_accel_value() {
//   // Map accel values
//   uint8_t output = constrain(map(accel_pedal_travel, 0, 100, 0, 4080), 0, 4080);
  
//   // Write accel output
//   analogWrite(analogWritePin, output);
// }


// calibration_reset()
// resets the calibration values after a button press, totally not needed in the future
// void calibration_reset() {
//   // Debounce button input, reset calibration values
//   if ((millis() - debounce_millis) > 500) {
//     accel_5v_max = 1000;
//     accel_5v_min = 500;
//     accel_3v_max = 1000;
//     accel_3v_min = 300;
//   }
//   // Set debounce millis
//   debounce_millis = millis();
// }


// print_state()
// log variables to console
void print_state() {
  Serial.print("Gas 5V: ");
  Serial.print(percent_5g);
  Serial.print("%  Analog:");
  Serial.print(analog_in_5acc);


  // Serial.print("  Gas 3.3V: ");
  // Serial.print(percent_3g);
  // Serial.print("%  Analog:");
  // Serial.print(analog_in_3acc);

  // Serial.print(" Implaus: ");
  // Serial.print(abs(percent_5g - percent_3g));

  Serial.print(" Combined Accel Pedal Travel: ");
  Serial.println(accel_pedal_travel); 

  // Serial.print("5V Max/Min: ");
  // Serial.print(accel_5v_max);
  // Serial.print(" ");
  // Serial.print(accel_5v_min);


  // Serial.print("  3.3v Max/Min: ");
  // Serial.print(accel_3v_max);
  // Serial.print(" ");
  // Serial.print(accel_3v_min);

  // Serial.print("     left:");
  // Serial.print(percent_3g);
  
  // Serial.print("     right: ");
  // Serial.println(percent_5g);

  Serial.print("Implausability check: ");
  Serial.println(brake_implausibility);

  delay(50);
}


// // read_accel_pedal_travel()
// // read both pedal values, convert to percentages and compare, set implausibility flag
// void read_accel_pedal_travel() {
//   analog_in_5acc = analogRead(analogPin5g); // 5V Gas INPUT
//   analog_in_3acc = analogRead(analogPin3g); // 3.3V Gas INPUT

//   if (analog_in_5acc > accel_5v_max) accel_5v_max = analog_in_5acc;
//   if (analog_in_5acc < accel_5v_min) accel_5v_min = analog_in_5acc;
//   if (analog_in_3acc > accel_3v_max) accel_3v_max = analog_in_3acc;
//   if (analog_in_3acc < accel_3v_min) accel_3v_min = analog_in_3acc;

//   percent_5g = constrain(map(analog_in_5acc, accel_5v_min, accel_5v_max, 0, 100), 0, 100);
//   percent_3g = constrain(map(analog_in_3acc, accel_3v_min, accel_3v_max, 0, 100), 0, 100);

//   // Pedal implausibility check
//   if (abs(percent_5g - percent_3g) < pedal_implausibility_thres) {
//     // Set accel_pedal_travel
//     accel_pedal_travel = constrain(((percent_5g + percent_3g) / 2), 0, 100);

//     // Make sure we passed 200ms timer threshold to reset pedal_implausibility
//     if (millis() - implausibility_timer > 200) {
//       pedal_implausibility = false;
//     }
//     return;
//   }

//   // Implausibility in 5v vs 3v values, set flag and timer
//   pedal_implausibility = true;
//   implausibility_timer = millis();
// }



// void read_eeprom() // read min/max values from eeprom; 
// {
//   accel_5v_max = 16 * EEPROM.read(0);
//   accel_5v_min = 16 * EEPROM.read(1);
//   accel_3v_max = 16 * EEPROM.read(2);
//   accel_3v_min = 16 * EEPROM.read(3);
//   Serial.println("EEPROM READ");
// }


// void write_to_eeprom() // write min/max values to eeprom; todo: find a way to store full 12bit number
// {
//   EEPROM.write(0, (accel_5v_max/16));
//   EEPROM.write(1, (accel_5v_min/16));
//   EEPROM.write(2, (accel_3v_max/16));
//   EEPROM.write(3, (accel_3v_min/16));
//   Serial.println("EEPROM UPDATED");
// }
