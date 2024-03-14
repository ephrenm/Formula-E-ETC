#include <Arduino_CAN.h>

static uint32_t const CAN_ID1 = 0x01; //CAN-ID

uint32_t msg_cnt1 = 0; //CAN Message Counter


#define analogPin5g A6  // 5V Gas Input
#define analogPin3g A7  // 3.3V Gas Input
#define analogWritePin A0

#define calibration_interrupt_pin 2 // Input Button, toggles the calibration state
#define throttle_indicator_pin 8 // Output LED, indicates if we outputting an accel value above
#define brake_state_pin 6
#define implausibility_indicator_pin 3 // Output LED, indicates if implausibility is detected

#define RTD_pin 5

int calibration_pin_state = 0; //Stores the state of calibration button

bool brakes_engaged = true;

// Variables for storing percentages mapped from analog readings
int accel_pedal_travel = 0; // was gasperc, combined accel percentages/2 for final pedal travel percentage
int brake_pedal_travel = 0; // was brakeperc, same as above

// Defining variables & initializing them as zero for potentiometer readings and calculated percentages
int analog_in_5acc = 0;
int analog_in_3acc = 0;
int percent_5g = 0;
int percent_3g = 0;

bool pedal_implausibility = false; // pedal_implausibility is the state that occurs when the position of two matching pedals do not agree
bool brake_implausibility = false; // brake_implausibility is the state that occurs when brake is engaged and accel > 25%
unsigned long implausibility_timer = 0;

// variable init for pedal calibration mins/maxs
int accel_5v_max = 1000;
int accel_5v_min = 500;
int accel_3v_max = 1000;
int accel_3v_min = 300;

int pedal_implausibility_thres = 10; // a percentage of the max allowable 3v to 5v accel sensor shift
int pedal_deadzone_thres = 5; // a percentage of throttle to ignore (0 to [this value])


unsigned long debounce_millis = 0; 



// setup()
// Before we enter the loop() below, this setup() function is run once
void setup() {
  Serial.begin(115200);

  pinMode(analogPin5g, INPUT);
  pinMode(analogPin3g, INPUT);
  // pinMode(gas_pin, OUTPUT);

  pinMode(calibration_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(calibration_interrupt_pin), calibration_reset, FALLING);

  pinMode(brake_state_pin, INPUT_PULLUP);
  pinMode(RTD_pin, INPUT);

  pinMode(throttle_indicator_pin, OUTPUT);
  pinMode(implausibility_indicator_pin, OUTPUT);

  pinMode(analogWritePin, OUTPUT);

  while (!Serial) { } //Wait until serial is opened

  //Create CAN object with a bit rate given.
  //Options include: 
  //BR_125k, BR_250k (250000), BR_500k (500000), BR_1000k (1000000)
  if (!CAN.begin(CanBitRate::BR_250k))
  {
    Serial.println("CAN.begin(...) failed.");
    for (;;) {} //Infinite loop if begin failed
  }
}


// loop()
// Main driver code, as long as the Arduino is on this code loops forever
void loop() {
  // Write LED statuses
  digitalWrite(throttle_indicator_pin, accel_pedal_travel >= pedal_deadzone_thres);
  digitalWrite(implausibility_indicator_pin, pedal_implausibility || brake_implausibility);

  calibration_pin_state = digitalRead(calibration_interrupt_pin);
  if (calibration_pin_state == 1) {
    calibration_reset();
  }

  // Read/update accel pedal travel, sets implausibility flag
  read_accel_pedal_travel();
  // Read/update brake pedal state, sets implausibility flag
  read_brake_pedal_travel();

  // Write accel value if conditions are met, otherwise set output to 0
  if (
    !pedal_implausibility // NO pedal implausibility
    && !brake_implausibility // NO break implausibility
    && accel_pedal_travel < pedal_deadzone_thres // Pedal travel above deadzone
    && digitalRead(RTD_pin) // Ready-to-drive signal is true
  ) {} 
  else {
    accel_pedal_travel = 0;
  }
  //Send Can Message
  send_can_msg();

  // Dump state to serial
  print_states();
}

// read_accel_pedal_travel()
// read both pedal values, convert to percentages and compare, set implausibility flag
void read_accel_pedal_travel() {
  analog_in_5acc = analogRead(analogPin5g); // 5V Gas INPUT
  analog_in_3acc = analogRead(analogPin3g); // 3.3V Gas INPUT

  if (analog_in_5acc > accel_5v_max) accel_5v_max = analog_in_5acc;
  if (analog_in_5acc < accel_5v_min) accel_5v_min = analog_in_5acc;
  if (analog_in_3acc > accel_3v_max) accel_3v_max = analog_in_3acc;
  if (analog_in_3acc < accel_3v_min) accel_3v_min = analog_in_3acc;

  percent_5g = constrain(map(analog_in_5acc, accel_5v_min, accel_5v_max, 0, 100), 0, 100);
  percent_3g = constrain(map(analog_in_3acc, accel_3v_min, accel_3v_max, 0, 100), 0, 100);

  // Pedal implausibility check
  if (abs(percent_5g - percent_3g) < pedal_implausibility_thres) {
    // Set accel_pedal_travel
    accel_pedal_travel = constrain(((percent_5g + percent_3g) / 2), 0, 100);

    // Make sure we passed 200ms timer threshold to reset pedal_implausibility
    if (millis() - implausibility_timer > 200) {
      pedal_implausibility = false;
    }
    return;
  }

  // Implausibility in 5v vs 3v values, set flag and timer
  pedal_implausibility = true;
  implausibility_timer = millis();
}

// read_brake_pedal_travel()
// read brake state, set implausibility flag
void read_brake_pedal_travel() {
  // Read brake state
  brakes_engaged = !digitalRead(brake_state_pin);

  // Brake implausibility check
  if (brakes_engaged && accel_pedal_travel > 5) {
    if (accel_pedal_travel > 25) {
      brake_implausibility = true;
    }
    // TODO: what happens if pedal travel is less than 25? condition falls out
  } else if (accel_pedal_travel < 6) {
    brake_implausibility = false;
  }
}

void send_can_msg() {
  uint8_t const msg_data1[] = {brake_pedal_travel,brake_implausibility,accel_pedal_travel,pedal_implausibility,0,0,0,0}; 
  memcpy((void *)(msg_data1 + 4), &msg_cnt1, sizeof(msg_cnt1));
  CanMsg const msg(CanStandardId(CAN_ID1), sizeof(msg_data1), msg_data1);

  int const rc = CAN.write(msg); 
  delay(100);
  if (rc < 0 ) 
  {
    Serial.print  ("CAN.write(...) failed with error code ");
    Serial.println(rc);
    //for (;;) { } //Infinite loop to never exit after error
  }
  msg_cnt1++;
}

// print_state()
// log variables to console
void print_states() {
  Serial.print("Gas 5V: ");
  Serial.print(percent_5g);
  Serial.print("%  Analog:");
  Serial.print(analog_in_5acc);
  Serial.print("  Gas 3.3V: ");
  Serial.print(percent_3g);
  Serial.print("%  Analog:");
  Serial.print(analog_in_3acc);

  Serial.print(" Implaus: ");
  Serial.print(abs(percent_5g - percent_3g));

  Serial.print(" Combined Accel Pedal Travel: ");
  Serial.println(accel_pedal_travel); 

  Serial.print("5V Max/Min: ");
  Serial.print(accel_5v_max);
  Serial.print(" ");
  Serial.print(accel_5v_min);
  Serial.print("  3.3v Max/Min: ");
  Serial.print(accel_3v_max);
  Serial.print(" ");
  Serial.print(accel_3v_min);

  Serial.print("     left:");
  Serial.print(percent_3g);
  
  Serial.print("     right: ");
  Serial.println(percent_5g);

  delay(50);
}

// calibration_reset()
// resets the calibration values after a button press, totally not needed in the future
void calibration_reset() {
  // Debounce button input, reset calibration values
  if ((millis() - debounce_millis) > 500) {
    accel_5v_max = 1000;
    accel_5v_min = 500;
    accel_3v_max = 1000;
    accel_3v_min = 300;
  }
  // Set debounce millis
  debounce_millis = millis();
}
