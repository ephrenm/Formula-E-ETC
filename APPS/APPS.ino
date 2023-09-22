// ETC Code v6 - Modified 9-1-23
// Changelog:
// 
// [TODO]
// Test and refine calibration, add calibration check!
// Add check for enable signal from master controller before outputting to I2C
// Add code to set a digital pin high whenever brake pedal is pressed (brake light activation for master controller)
// EV.5.7 - Brake Pedal Plausibility Check (BPPC) - mechanical brakes engaged + 25% pedal travel
// Out-of-range values from APPS to Arduino & between Arduino and motor controller - T.4.2.10 & T.4.3.4
// Once brake pedal assy finalized, write min/maxs to EEPROM so calibration not necessary every time

//Implausibility Occurs When:
//paired pedals disagree by 10%, when this occurs for more than 100 millisec, power to motor must be cut
//When brakes engaged and accel_pedal_travel > 25%
//in second case, power to motor must be completely cut until accel reads less than 5% w or w/o brakes engaged
//any failure of the APPS or wiring must be detectable by controller and treated like an implausibility

#include <EEPROM.h>

// Pins that potentiometers are connected to
#define analogPin5g A2  // 5V Gas Input
#define analogPin3g A1  // 3.3V Gas Input
#define analogPin5s A4  // 5V Stop Input
#define analogPin3s A5 // 3.3V Stop Input
#define analogWritePin A0

#define button_interrupt_pin 2
#define gaspin 6
#define brake_state_pin 8
#define calibration_toggle_pin 7
#define implausibility_pin 9

bool calibration_state = false;
bool brakes_engaged = true;

// Variables for storing percentages mapped from analog readings
int accel_pedal_travel; //was gasperc, combined accel percentages/2 for final pedal travel percentage
int brake_pedal_travel; //was brakeperc, same as above

// Defining variables & initializing them as zero for potentiometer readings and calculated percentages
int analog_in_5acc = 0; //was analog_5g
int analog_in_3acc = 0;
int analog_in_5brake = 0; //was analog_5s
int analog_in_3brake = 0;
int percent_5g = 0;
int percent_3g = 0;
int percent_5s = 0;
int percent_3s = 0;

bool pedal_implausibility = false;
bool temp_pedal_implausibility = false;
bool brake_implausibility = false;
unsigned long implausibility_timer = 0;


//variable init for pedal calibration mins/maxs
int accel_5v_max = 0;
int accel_5v_min = 100;
int accel_3v_max = 0;
int accel_3v_min = 100;
int brake_5v_max = 0;
int brake_5v_min = 100;
int brake_3v_max = 0;
int brake_3v_min = 100;

unsigned long debounce_millis = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(analogPin5g, INPUT);
  pinMode(analogPin3g, INPUT);
  pinMode(analogPin5s, INPUT);
  pinMode(analogPin3s, INPUT);
  pinMode(gaspin, OUTPUT);
  pinMode(brake_state_pin, OUTPUT);

  pinMode(brake_state_pin, INPUT_PULLUP);

  pinMode(button_interrupt_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_interrupt_pin), calibration_state_toggle, FALLING);

  pinMode(calibration_toggle_pin, OUTPUT);
  pinMode(implausibility_pin, OUTPUT);

  pinMode(analogWritePin, OUTPUT);
  analogWriteResolution(12);

  read_eeprom();
}

void loop() {
  digitalWrite(calibration_toggle_pin, calibration_state);
  digitalWrite(implausibility_pin, pedal_implausibility || brake_implausibility);
  
  if (calibration_state)
  {
    while(calibration_state) calibrate_pedals();
    write_to_eeprom();
  }

  if(get_accel_pedal_travel()) 
  {
    pedal_implausibility = false;
    temp_pedal_implausibility = false;
  } else {
    if(temp_pedal_implausibility = false) {
      temp_pedal_implausibility = true;
      implausibility_timer = millis();
    } else {
      if (millis() - implausibility_timer > 100)
      {
        pedal_implausibility = true;
      }
    }
  }

  get_brake_pedal_travel();
  check_brake_implausibility();

  if (!pedal_implausibility || !brake_implausibility)
  {
    writeAccelValue();
  } else {
    analogWrite(analogWritePin, 0);
  }
  //print_state();
  Serial.println(accel_pedal_travel);
  if(brakes_engaged)
  {
    Serial.println("Brakes engaged");
  }
}

void check_brake_implausibility()
{
  if(brakes_engaged && accel_pedal_travel > 5) {
    if(accel_pedal_travel > 25) 
    {
      brake_implausibility = true;
    }
  } else if(accel_pedal_travel < 6) {
    brake_implausibility = false;
  }
}

void read_eeprom() // read min/max values from eeprom
{
  accel_5v_max = 16 * EEPROM.read(0);
  accel_5v_min = 16 * EEPROM.read(1);
  accel_3v_max = 16 * EEPROM.read(2);
  accel_3v_min = 16 * EEPROM.read(3);
  Serial.println("EEPROM READ");
}

void write_to_eeprom() // write min/max values to eeprom
{
  EEPROM.write(0, (accel_5v_max/16));
  EEPROM.write(1, (accel_5v_min/16));
  EEPROM.write(2, (accel_3v_max/16));
  EEPROM.write(3, (accel_3v_min/16));
  Serial.println("EEPROM UPDATED");
}

void writeAccelValue()  //transform pedal travel % to usable DAC 12-bit value
{
  int output = constrain(map(accel_pedal_travel, 0, 100, 0, 4080),0, 4080);
  //Serial.println(output);
  analogWrite(analogWritePin, output);
}

/*void check_and_set_implausibility() //check and set implaus; unfinished
{
  static unsigned long startimptime = 0;

  if (pedal_implausibility == false) {               // check if the variable outside the loop is false (named implausibility)
      pedal_implausibility = true;                     // If it is currently false, then set it to true, upon which it will stay true until the error stops
      startimptime = millis();                   // And then call millis() which records the time when implausibility started
    } else if (millis() - startimptime > 100) {  // Then, check if more than 100ms has passed and if so, do everything when implausibility happens
      Serial.println("Implausibility Occured");
      accel_pedal_travel = 0;
      brake_pedal_travel = 0;
    }
}*/

void calibration_state_toggle()
{
  //debounce button input and toggle calibration state //
  if ((millis() - debounce_millis) > 500) {
    calibration_state = !calibration_state;
  }

  debounce_millis = millis();
}

void calibrate_pedals() //set min/max pedal vals based on measured travel, need to add checks to ensure min/max vals have a reasonable difference
{
  Serial.println("you are now calibrating");
  if (analogRead(analogPin5g) > accel_5v_max) accel_5v_max = analogRead(analogPin5g);
  if (analogRead(analogPin5g) < accel_5v_min) accel_5v_min = analogRead(analogPin5g);
  if (analogRead(analogPin3g) > accel_3v_max) accel_3v_max = analogRead(analogPin3g);
  if (analogRead(analogPin3g) < accel_3v_min) accel_3v_min = analogRead(analogPin3g);
  if (analogRead(analogPin5s) > brake_5v_max) brake_5v_max = analogRead(analogPin5s);
  if (analogRead(analogPin5s) < brake_5v_min) brake_5v_min = analogRead(analogPin5s);
  if (analogRead(analogPin3s) > brake_3v_max) brake_3v_max = analogRead(analogPin3s);
  if (analogRead(analogPin3s) < brake_3v_min) brake_3v_min = analogRead(analogPin3s);
}

void print_state() //print state
{
  //Gas Pedal
  Serial.print("Gas 5V: ");
  Serial.print(percent_5g);
  Serial.print("%  Analog:");
  Serial.print(analog_in_5acc);
  Serial.print("  Gas 3.3V: ");
  Serial.print(percent_3g);
  Serial.print("%  Analog:");
  Serial.print(analog_in_3acc);

  // Brake Pedal
  Serial.print("  Brake 5V Travel: ");
  Serial.print(percent_5s);
  Serial.print("%  Analog:");
  Serial.print(analog_in_5brake);
  Serial.print("  Brake 3.3V Travel: ");
  Serial.print(percent_3s);
  Serial.print("%  Analog:");
  Serial.println(analog_in_3brake);

  Serial.print("Combined Accel Pedal Travel: ");
  Serial.print(accel_pedal_travel); 
  Serial.print("    Combined Brake Pedal Travel: ");
  Serial.println(brake_pedal_travel);
}

bool get_accel_pedal_travel () //read both pedal values, convert to percentages and compare, return false if implaus occurs
{
  bool return_state = false;

  analog_in_5acc = analogRead(analogPin5g);  // 5V Gas INPUT
  analog_in_3acc = analogRead(analogPin3g);  // 3.3V Gas INPUT
  percent_5g = constrain(map(analog_in_5acc, accel_5v_min, accel_5v_max, 0, 100), 0, 100);
  percent_3g = constrain(map(analog_in_3acc, accel_3v_min, accel_3v_max, 0, 100), 0, 100);

  // Gas pedal implausibility check
  if (abs(percent_5g - percent_3g) < 10.0) {
    return_state = true;
    accel_pedal_travel = constrain(((percent_5g + percent_3g) / 2), 0, 100);  // Since no implausibility, taking the average of the two percentages
  } else {
    return_state = false;
  }

  return return_state;
}

void get_brake_pedal_travel () //read both pedal values, convert to percentages and compare, return false if implaus occurs
{
  brakes_engaged = !digitalRead(brake_state_pin);
}


