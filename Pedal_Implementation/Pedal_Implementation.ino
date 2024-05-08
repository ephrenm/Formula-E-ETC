#include <Arduino_CAN.h>

uint8_t counter_5v = 0;
uint8_t counter_3v = 0;
uint8_t counter_Stuck_High = 0;
uint8_t counter_Stuck_Low1 = 0;
uint8_t counter_Stuck_Low2 = 0;
uint8_t counter_Stuck_Low3 = 0;
uint8_t counter_Stuck_Low4 = 0;
uint16_t last_updated_low = millis();
uint16_t last_updated_high = millis();

bool piecewise_implementation = false;


//PIECEWISE APPROXIMATION PARAMETERS
#define x1 256
#define y1 64
#define y2 320
#define y3 832

#define analogPin5g A0  // 5V Gas Input
#define analogPin3g A1  // 3.3V Gas Input
// #define analogWritePin A0

#define calibration_interrupt_pin D2 // Input Button, toggles the calibration state
#define brake_state_pin D12
#define implausibility_led D6 // Output LED, indicates if implausibility is detected
#define throttle_led D7 // Output LED, indicates if we outputting an accel value above
// #define RTD_pin 5

//GLOBAL PARAMETERS
#define pedal_implausibility_thres 10   //mutual shift in terms of percentage of full scale
#define pedal_deadzone_thres_5v 5       // a percentage of throttle to ignore (0 to [this value])
#define pedal_deadzone_thres_3v 5       // a percentage of throttle to ignore (0 to [this value])

//S=3.3V
//delta= 3.3/1023= 

bool brakes_engaged = true;
// Variables for storing percentages mapped from analog readings
uint8_t brake_pedal_travel; // was brakeperc, same as above

// Defining variables & initializing them as zero for potentiometer readings and calculated percentages
uint32_t analog_in_5Vacc = 0;
uint32_t analog_in_3Vacc = 0;

uint8_t percent_5g = 0;
uint8_t percent_3g = 0;

uint8_t accel_pedal_travel; // Combined mean accel percentages of 5V & 3V

bool pedal_implausibility = false; // pedal_implausibility is the state that occurs when the position of two matching pedals do not agree
bool brake_implausibility = false; // brake_implausibility is the state that occurs when brake is engaged and accel > 25%

static unsigned long pedal_implausibility_timer = 0;
static unsigned long brake_implausibility_timer = 0;

// variable init for pedal calibration mins/maxs
uint8_t accel_5v_max = 0;
uint8_t accel_5v_min = 1000;
uint8_t accel_3v_max = 0;
uint8_t accel_3v_min = 1000;

unsigned long debounce_millis = 0;

static uint32_t const CAN_ID = 0x01; //CAN ID
uint32_t msg_cnt = 0; //Can msg counter


// setup()
// Before we enter the loop() below, this setup() function is run once
void setup() {
  Serial.begin(115200);
  pinMode(analogPin5g, INPUT);
  pinMode(analogPin3g, INPUT);

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
  if (!CAN.begin(CanBitRate::BR_250k)) {
      Serial.println("CAN.begin(...) failed.");
      for (;;) {}
    }
}

void loop() {
  //TODO something has to RESET implausibility FLAGS

  // uint8_t calibration_pin_state = digitalRead(calibration_interrupt_pin);
  // if (calibration_pin_state == 1) {
  //   calibration_reset();
  // }
  // Read/update accel pedal travel && sets implausibility flag
  read_accel_pedal_travel();
  // Read/update brake pedal state && sets implausibility flag
  read_brake_pedal_travel();

  //Write LED statuses 
  digitalWrite(throttle_led, accel_pedal_travel >= pedal_deadzone_thres_5v);
  digitalWrite(implausibility_led, pedal_implausibility || brake_implausibility);

  send_can_msg();  //CAN wite func

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

  print_state(); // Dump state to serial
}

void send_can_msg() {
  uint8_t can_msg_out[] = {0,accel_pedal_travel}; 
  can_msg_out[0] = brake_implausibility | pedal_implausibility<<1;

  memcpy((void *)(can_msg_out + 4), &msg_cnt, sizeof(msg_cnt));

  CanMsg const msg(CanStandardId(CAN_ID), sizeof(can_msg_out), can_msg_out);
  int const rc = CAN.write(msg); 

  if (rc < 0)
  {
    Serial.print  ("CAN.write(...) failed with error code ");
    Serial.print(rc);
    Serial.println(" ,-60003 is failed to send message");
  }
  msg_cnt++;
}


// read brake state, set implausibility flag
void read_brake_pedal_travel() {
    // Read brake state
    brakes_engaged = !digitalRead(brake_state_pin);

  // Brake implausibility check
  if (brakes_engaged && accel_pedal_travel > 25) {
      brake_implausibility = true;
      brake_implausibility_timer = millis();
    // TODO: what happens if pedal travel is less than 25? condition falls out
  } 
  else if (accel_pedal_travel < 6 && millis() - brake_implausibility_timer > 200) {
      // Make sure we passed 200ms timer threshold to reset pedal_implausibility
      brake_implausibility = false;
  }
}

void check_pedal_stuck() {
  if (accel_pedal_travel > 0  && accel_pedal_travel <= 1) {
        counter_Stuck_Low1++;
  }
  else if (accel_pedal_travel > 1  && accel_pedal_travel <= 2) {
        counter_Stuck_Low2++;
  }
  else if (accel_pedal_travel > 2  && accel_pedal_travel <= 3) {
        counter_Stuck_Low3++;
  }
  else if (accel_pedal_travel > 3  && accel_pedal_travel <= 4) {
        counter_Stuck_Low4++;
  }

  /*counter -> milisec: 2 -> 57, 3 -> 114, 4 -> 171, 5 -> 228, 6 -> 285, 7 -> 342, 8 -> 399, 9 -> 456 10 -> 502, 
  11 -> 560, 12 -> 616, 13 -> 683, 14 -> 741, 15 -> 798, 16 -> 855, 17 -> 912, 18 -> 969, 19 -> 1026, 20 -> 1083*/

  //Stuck low
  if ((counter_Stuck_Low1 == 5 ||  counter_Stuck_Low2 == 5 || counter_Stuck_Low3 == 5 ||
    counter_Stuck_Low4 == 5) && (millis() - last_updated_low) > 1000) { 
    if (accel_3v_min < 92 || accel_5v_min < 48) {
      accel_3v_min += 4;
      accel_5v_min += 4;
      last_updated_low = millis();
    }
  }
  //Stuck high
  else if (accel_pedal_travel >= 90 && accel_pedal_travel <= 99) {
    counter_Stuck_High++;
    if (counter_Stuck_High == 8 && (millis() - last_updated_high) > 1000) {
      accel_3v_max -= 3;
      accel_5v_max -= 3;
      last_updated_high = millis();
    }
  } 
  else {
    counter_Stuck_Low1 = 0;
    counter_Stuck_Low2 = 0;
    counter_Stuck_Low3 = 0;
    counter_Stuck_Low4 = 0;
    counter_Stuck_High = 0;
  }
}

// read_accel_pedal_travel()
// read both pedal values, convert to percentages and compare, set implausibility flag
void read_accel_pedal_travel() {
  uint8_t analog_tmp;
  analog_in_5Vacc=0;
  analog_in_3Vacc=0;
  counter_3v=0;
  counter_5v=0;
  
  for(int i=0;i<32;i++){
    analog_tmp=analogRead(analogPin5g);
    //Count for analog accuracy when finding a new min/max
    if (analog_tmp > accel_5v_max + pedal_deadzone_thres_5v) counter_5v++;
    if (analog_tmp < accel_5v_min - pedal_deadzone_thres_5v) counter_5v++;
    analog_in_5Vacc += analog_tmp; // 5V Gas INPUT

    analog_tmp=analogRead(analogPin3g);
    //Count for analog accuracy when finding a new min/max
    if (analog_tmp > accel_3v_max + pedal_deadzone_thres_3v)counter_3v++;
    if (analog_tmp < accel_3v_min - pedal_deadzone_thres_3v)counter_3v++;
    analog_in_3Vacc += analog_tmp;
    delayMicroseconds(100);     //100us delay
    //Total runtime delay 3.2 ms
  }
  analog_in_5Vacc=analog_in_5Vacc >> 5; //Gets avg dividing by 32
  analog_in_3Vacc=analog_in_3Vacc >> 5; //Gets avg dividing by 32

  if (analog_in_5Vacc > accel_5v_max + pedal_deadzone_thres_5v && counter_5v > 28) accel_5v_max = analog_in_5Vacc;
  if (analog_in_5Vacc < accel_5v_min - pedal_deadzone_thres_5v && counter_5v > 28) accel_5v_min = analog_in_5Vacc;
  if (analog_in_3Vacc > accel_3v_max + pedal_deadzone_thres_3v && counter_3v > 28) accel_3v_max = analog_in_3Vacc;
  if (analog_in_3Vacc < accel_3v_min - pedal_deadzone_thres_3v && counter_3v > 28) accel_3v_min = analog_in_3Vacc;

  //percentage mapping of throttles
  percent_5g = constrain(map(analog_in_5Vacc, accel_5v_min, accel_5v_max, 0, 100), 0, 100);
  percent_3g = constrain(map(analog_in_3Vacc, accel_3v_min, accel_3v_max, 0, 100), 0, 100);

  // Set accel_pedal_travel by avg 5v & 3v 
  accel_pedal_travel = (percent_5g + percent_3g)>>1; 

  //Checks if pedal is stuck at 1-4 or 96-99 ajusting to 0 or 100 respectively
  check_pedal_stuck();

  // if (piecewise_implementation) {
  //   piecewise_throttle1(&analog_in_5Vacc); //Piece wise attempt #1
  // }

  // Pedal implausibility check
  if (abs(percent_5g - percent_3g) > pedal_implausibility_thres) { 
    pedal_implausibility = true;
    pedal_implausibility_timer = millis(); //sample the implausibility time
  }
  else if (millis() - pedal_implausibility_timer > 200) { //200ms timer threshold to reset pedal_implausibility
    pedal_implausibility = false;
  }
}

//Needs to be changed. We cannot manipulate the data to be higher than it actually is
//only for it to be decreased. New approach is needed.
void piecewise_throttle1(uint32_t* pedal_acc ) { // 10 bit dac
  uint16_t x = ((accel_5v_max-accel_5v_min)>>2); //last 8 bit
  Serial.print("\n Throttle 5v Before:");
  Serial.println(*pedal_acc);
  
  if(*pedal_acc < x+accel_5v_min){ //SLOPE 1 - 1/4 pedal_acc
    (*pedal_acc) = ((*pedal_acc)>>2);
  }
  else if(*pedal_acc < 2*x+accel_5v_min){ //SLOPE 2 - 1/4*x + (ped-x)/2
    (*pedal_acc) = ((x>>2) + (((*pedal_acc)-x)>>1));              
  }
  else if(*pedal_acc < 4*x+accel_5v_min){ //SLOPE 3 - 1/4*x + 1/2*x + ped-2*x
    (*pedal_acc) = (x>>2) + (x>>1) + ((*pedal_acc)-2*x);
  }
  else{
    (*pedal_acc) = (x>>2) + (x>>1)  + x + ((*pedal_acc)-3*x)*2; //SLOPE 4 - 1/4*x + 1/2*x + x + (ped-3*x)*2
  }
  (*pedal_acc) += accel_5v_min;

  Serial.print("  Throttle 5v After:");
  Serial.println(*pedal_acc);
}

// print_state()
// log variables to console
void print_state() {
  Serial.print("Gas 5V: ");
  Serial.print(percent_5g);
  Serial.print("%  Analog:");
  Serial.print(analog_in_5Vacc);
  Serial.print("  Gas 3.3V: ");
  Serial.print(percent_3g);
  Serial.print("%  Analog:");
  Serial.print(analog_in_3Vacc);

  Serial.print(" Pedal mismatch: ");
  Serial.print(abs(percent_5g - percent_3g));

  Serial.print(" Combined Accel Pedal Travel: ");
  Serial.println(accel_pedal_travel); 

  Serial.print(" 5V Max/Min: ");
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
  Serial.print(percent_5g);

  Serial.print(" Implausability check: ");
  Serial.println(brake_implausibility);

  delay(50);
}
