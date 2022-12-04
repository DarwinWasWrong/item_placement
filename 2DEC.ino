/*

when testing here, be aware that there are limit switch positions that
are not possible in real life.
Like leaving the limit switch on when you are testing the home
If you are moving from home, then switch home limit OFF
Then switch limit ON
AND
If you are moving from limit, then switch  limit OFF
Then switch home ON
failure to do so - isnt real life and will result in strange behaviour


The LCD display is a bit off - but in this test could not be bothered


E Series Nema 23 Bipolar 1.8deg 3.0 Nm(425oz.in) 4.2A 57x57x113mm 4 Wires

    Manufacturer Part Number: 23HE45-4204S
    Number of phase: 2
    Step Angle: 1.8 deg
    Holding Torque: 3.0 Nm(425oz.in)
    Rated Current/phase: 4.2 A
    Phase Resistance: 0.9 ohms± 10%
    Inductance: 3.8 mH ± 20%(1KHz)

Step Angle: 1.8 deg
200 steps per revolution

@ 1/2 stepping, that will give you 400 steps per revolution
That is set on the Digital Microstep driver DM542 Stepper Motor Controller
Which we do not have here.

*/




#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include "EasyButton.h"

LiquidCrystal_I2C LCD = LiquidCrystal_I2C(0x27, 16, 2);

#define DEBUG true  //set to true for debug output, false for no debug output
#define DEBUG_SERIAL if(DEBUG)Serial
#define DEBUG_PAUSE if(DEBUG)millisdelay
#define DEBUG_LCD if(DEBUG)displayLCD

volatile int debug_heading_counter = 0;
volatile int change_check;
volatile int previous_change_check;
volatile int thechange;
volatile bool state_led;


// MACHINE switch inputs
//hardware interupt
#define TABLE_BAR_LIMIT_PIN 3
#define TABLE_BAR_HOME_PIN 2
#define PRODUCT_SHELF_HOME_PIN 18
#define PRODUCT_SHELF_LIMIT_PIN 19

//pin change interupt
#define OPERATOR_EMERGENCY_STOP_INPUT_PIN 10
#define PRODUCT_IN_PLACE_INPUT_PIN        11


// OPERATOR SWITCHES
// Button code
#define OPERATOR_START_SWITCH_PIN    A12
#define OPERATOR_BACK_SWITCH_PIN     A11
#define OPERATOR_UP_SWITCH_PIN       A10
#define OPERATOR_FORWARD_SWITCH_PIN  A9
#define OPERATOR_DOWN_SWITCH_PIN     A8


// Vertical Stepper
#define PRODUCT_SHELF_STEPPER_CONT_EN   23
#define PRODUCT_SHELF_STEPPER_CONT_STEP 27
#define PRODUCT_SHELF_STEPPER_CONT_DIR  25


// Horizonal Stepper
#define TABLE_BAR_STEPPER_CONT_EN     29
#define TABLE_BAR_STEPPER_CONT_STEP   31
#define TABLE_BAR_STEPPER_CONT_DIR    33

// Misc  outputs
#define  EMERGENCY_STOP_LED 12

// Stepper constants
const int stepsPerRevolution = 200 ;              // set for steppers
int initial_present_position;                     // general position variable
int stepper_delay = 2  ;                          // ms for move of 1

// flipper display
uint8_t backslash[8] = {
  0b00000,
  0b10000,
  0b01000,
  0b00100,
  0b00010,
  0b00001,
  0b00000,
  0b00000
};

// we use special character 0 here
char flipper[] = {'/', '-', 0, '|'};







// Boolean flags for status product_in_place sensor
volatile boolean stock_in_place                    = false;
volatile boolean previous_stock_in_place           = true;

// Boolean flags for status product_shelf
volatile boolean product_shelf_home_calibrated      = false;
volatile boolean product_shelf_limit_calibrated     = false;
volatile boolean product_shelf_home                 = false;
volatile boolean product_shelf_limit                = false;



// Boolean flags for status table_bar
volatile boolean table_bar_home_calibrated         = false;
volatile boolean table_bar_limit_calibrated        = false;
volatile boolean table_bar_home                    = false;
volatile boolean table_bar_limit                   = false;


// operator buttons
volatile boolean emergency_stop                    = false;
volatile boolean start_pushed                      = false;
volatile boolean table_forward_pushed              = false;
volatile boolean table_backward_pushed             = false;
volatile boolean shelf_up_pushed                   = false;
volatile boolean shelf_down_pushed                 = false;
volatile boolean cycle_not_running                 = true;

// stepper position variables
volatile int shelf_stepper_position = 0  ;                 // Shelf position
volatile int table_stepper_position = 0  ;                 // Table position
volatile int shelf_stepper_home_position = 0;              // assume zero after home
volatile int table_stepper_home_position = 0;              // assume zero after home
volatile int shelf_stepper_limit_position = 10000;         // Some +ve value set later
volatile int table_stepper_limit_position = 10000;         // Some +ve value set later


//testing variables and leds
volatile boolean completed_tests = false;
int test_counter = 0;
int test_counter_previous = 0;
int logical_table_bar_HOME_led = 41;
int logical_table_bar_LIMIT_led = 43;
int logical_product_shelf_HOME_led = 45;
int logical_product_shelf_LIMIT_led = 47;
int test_delay = 2000;


// Setup steppers
AccelStepper TABLE_BAR(1, TABLE_BAR_STEPPER_CONT_STEP, TABLE_BAR_STEPPER_CONT_DIR);
AccelStepper PRODUCT_SHELF(1, PRODUCT_SHELF_STEPPER_CONT_STEP, PRODUCT_SHELF_STEPPER_CONT_DIR);

// Operator normal switches/buttons
EasyButton  startButton(OPERATOR_START_SWITCH_PIN);
EasyButton  tablebackButton(OPERATOR_BACK_SWITCH_PIN  );
EasyButton  tableforwardButton( OPERATOR_FORWARD_SWITCH_PIN );
EasyButton  shelfupButton(OPERATOR_UP_SWITCH_PIN );
EasyButton  shelfdownButton( OPERATOR_DOWN_SWITCH_PIN);



void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


void pciDisable() // set up interrupts for each pin (This is called from Setup function)
{
  PCMSK1 = 0; // Atmel datasheet 13.2.7 PCMSK1 – Pin Change Mask Register 1
  //disable pinChange interrupts
  PCICR = 0; //disable pin change interrupt register
  PCIFR = 0; //disable pin change interript flags
}


void pciEnable() {
  // Attach the new PinChangeInterrupt
  // supported pins 10, 11, 12, 13, 14, 15, 50, 51, 52, 53,
  // A8 (62), A9 (63), A10 (64), A11 (65), A12 (66),
  // A13 (67), A14 (68), A15 (69).
  pciSetup(OPERATOR_EMERGENCY_STOP_INPUT_PIN);
  pciSetup(PRODUCT_IN_PLACE_INPUT_PIN);

}



void setup() {

  Serial.begin(9600);
  Serial.println("Start called");

  // Stepper Horizonal
  TABLE_BAR.setEnablePin(TABLE_BAR_STEPPER_CONT_EN);
  TABLE_BAR.setPinsInverted(true, false, true);
  TABLE_BAR.enableOutputs();
  TABLE_BAR.setMaxSpeed(2000);
  TABLE_BAR.setSpeed(2000);
  TABLE_BAR.setAcceleration(10);

  // Stepper Vertical
  PRODUCT_SHELF.setEnablePin(PRODUCT_SHELF_STEPPER_CONT_EN);
  PRODUCT_SHELF.setPinsInverted(true, false, true);
  PRODUCT_SHELF.enableOutputs();
  PRODUCT_SHELF.setMaxSpeed(2000);
  PRODUCT_SHELF.setSpeed(200);
  PRODUCT_SHELF.setAcceleration(10);

  // set pin modes - switches have pull down resistors - product_in_place switch active HIGH/LOW
  pinMode(PRODUCT_IN_PLACE_INPUT_PIN, INPUT);     // High normally - low on switched -
  // we use inverted value to get logic levels right way around

  pinMode(TABLE_BAR_LIMIT_PIN, INPUT);            // low normally - high on switched
  pinMode(TABLE_BAR_HOME_PIN, INPUT);             // low normally - high on switched

  pinMode(PRODUCT_SHELF_HOME_PIN, INPUT);         // low normally - high on switched
  pinMode(PRODUCT_SHELF_LIMIT_PIN, INPUT);        // low normally - high on switched

  pinMode(OPERATOR_START_SWITCH_PIN, INPUT);      // low normally - high on switched
  pinMode(OPERATOR_FORWARD_SWITCH_PIN, INPUT);    // low normally - high on switched
  pinMode(OPERATOR_BACK_SWITCH_PIN, INPUT);       // low normally - high on switched
  pinMode(OPERATOR_UP_SWITCH_PIN, INPUT);         // low normally - high on switched
  pinMode(OPERATOR_DOWN_SWITCH_PIN, INPUT);       // low normally - high on switched



  pinMode(EMERGENCY_STOP_LED, OUTPUT);        // LED or LIGHT for emergency stop condition

  // pin mode on the stepper enables - most likely not required
  pinMode(TABLE_BAR_STEPPER_CONT_EN, OUTPUT);
  pinMode(PRODUCT_SHELF_STEPPER_CONT_EN, OUTPUT);


  // TESTING LOGIC LEDS - NOT IN PRODUCTION
  pinMode(logical_table_bar_HOME_led, OUTPUT);
  pinMode(logical_table_bar_LIMIT_led, OUTPUT);
  pinMode(logical_product_shelf_HOME_led, OUTPUT);
  pinMode(logical_product_shelf_LIMIT_led, OUTPUT);

  // Attach the PinChangeInterrupts
  pciEnable();


  // SET THE LIMIT SWITCHES HARDWARE INTERUPTS

  attachInterrupt(digitalPinToInterrupt(PRODUCT_SHELF_HOME_PIN), shelf_homeInterupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PRODUCT_SHELF_LIMIT_PIN), shelf_limitInterupt, CHANGE);

  attachInterrupt(digitalPinToInterrupt(TABLE_BAR_HOME_PIN), table_homeInterupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TABLE_BAR_LIMIT_PIN), table_limitInterupt, CHANGE);



  //operator Buttons
  startButton.onPressed(startButtonCallback);
  tablebackButton.onPressed(tablebackButtonCallback);
  tableforwardButton.onPressed(tableforwardButtonCallback);
  shelfupButton.onPressed(shelfupButtonCallback);
  shelfdownButton.onPressed(shelfdownButtonCallback);


  /* -------------------------------------------
    real startup sequence for safety
    -------------------------------------------
  */

  // ENSURE STEPPERS ARE NOT RUNNING
  // send stop and then run that command
  TABLE_BAR.stop();
  TABLE_BAR.run();
  PRODUCT_SHELF.stop();
  PRODUCT_SHELF.run();
  // read the EMERGENCY STOP switch - high means its still on
  emergency_stop = digitalRead(OPERATOR_EMERGENCY_STOP_INPUT_PIN);

  // START THE SCREEN
  LCD.init();
  LCD.backlight();
  LCD.setCursor(0, 0);


  // CHECK TO MAKE SURE the switch is off
  if (emergency_stop)
  {
    LCD.setCursor(0, 0);
    LCD.print(" EMERGENCY STOP     ");
    LCD.setCursor(0, 1);
    LCD.print("Power Reset          ");
    digitalWrite(EMERGENCY_STOP_LED, HIGH);
    while (1);
  }
  // we have reached here, so we wait for the operator to hit the start button
  LCD.clear();
  millisdelay(1000, true);
  LCD.setCursor(0, 0);
  LCD.print("READY TO TEST     ");
  buttonrefresh();
}

void buttonrefresh()
{
  startButton.read();
  tablebackButton.read();
  tableforwardButton.read();
  shelfupButton.read();
  shelfdownButton.read();
}



void loop() {

  // update the putton stattus
  buttonrefresh();
  // set LCD display default
  LCD.setCursor(0, 0);
  LCD.print("START-> ");
  LCD.setCursor(8, 0);

  if ( !completed_tests)
  {
    if (test_counter == 0)
    {
      LCD.print("Startup");
      LCD.setCursor(0, 1);
      LCD.print("TABLE HOME   ");


    }


   


    if ( test_counter_previous != test_counter)
    {
      switch (test_counter)
      {

        case 1:
          cycle_not_running = false;
          
          move_table_bar_home_on_startup();
          //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
         
       
          // next test display
          LCD.print("Startup");
          LCD.setCursor(0, 1);
          LCD.print("SHELF HOME      ");
          break;


        case 2:
          cycle_not_running = false;

          move_shelf_home_on_startup();
          
          //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
         

          // next test display
          LCD.print("Startup");
          LCD.setCursor(0, 1);
          LCD.print("TABLE LIMIT   ");
          break;

        case 3:
          cycle_not_running = false;

          move_table_bar_forward();
          //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
          
          // next test display
          LCD.print("CYCLE");
          LCD.setCursor(0, 1);
          LCD.print("SHELF LIMIT     ");
          break;




        case 4:
          cycle_not_running = false;

          move_product_shelf_up();
          
          //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");
          
          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
          
          // next test display
          LCD.print("CYCLE ");
          LCD.setCursor(0, 1);
          LCD.print("SHELF HOMEx    ");
          break;




        case 5:
          cycle_not_running = false;

          move_product_shelf_home();

          //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
          
          // next test display
          LCD.print("CYCLE ");
          LCD.setCursor(0, 1);
          LCD.print("SHELF LIMIT   ");
          break;


        case 6:
          cycle_not_running = false;

          move_product_shelf_up();
 
           //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
  
          // next test display
          LCD.print("CYCLE ");
          LCD.setCursor(0, 1);
          LCD.print("TABLE HOME   ");
          break;





        case 7:
          cycle_not_running = false;

          move_table_bar_home();

          //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
          
          
                   // next test display
          LCD.print("CYCLE ");
          LCD.setCursor(0, 1);
          LCD.print("TABLE LIMIT ");
          
          break;




        case 8:
          cycle_not_running = false;


          move_table_bar_forward();
 
           //result 
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
          
          // next test display
          LCD.print("STARTUP   ");
          LCD.setCursor(0, 1);
          LCD.print("END  ");
          
          break;

       case 9:



          cycle_not_running = false;
          startup();
          LCD.setCursor(12, 1);
          LCD.print("OK  ");

          test_counter_previous = test_counter;
          cycle_not_running = true;
          millisdelay(test_delay, true);
   

        case 10:

          LCD.clear();
          LCD.setCursor(0, 0);
          LCD.print("TESTS COMPLETED");
          LCD.setCursor(0, 1);
          LCD.print("Start TO REDO     ");
          millisdelay(test_delay, true);
          completed_tests = false;
          test_counter = 0;
          break;


      }

    }





  }
  // test leds - not in production
  digitalWrite(logical_product_shelf_LIMIT_led, product_shelf_limit);
  digitalWrite(logical_product_shelf_HOME_led, product_shelf_home);
  digitalWrite(logical_table_bar_LIMIT_led, table_bar_limit);
  digitalWrite(logical_table_bar_HOME_led, table_bar_home);

}


void tablebackButtonCallback()
{
  if (cycle_not_running)
  {
    // move until home limit or release
    cycle_not_running = true;
    TABLE_BAR.stop();
    TABLE_BAR.run();
    Serial.println("tablebackButtonCallback");
  }
}


void tableforwardButtonCallback()
{ 
  if (cycle_not_running)
  {
    // move until front limit or release
    cycle_not_running = true;
    TABLE_BAR.stop();
    TABLE_BAR.run();
    Serial.println("tableforwardButtonCallback");
  }
}

void shelfupButtonCallback()
{
  if (cycle_not_running)
  {
    // move until up limit or release
    PRODUCT_SHELF.stop();
    PRODUCT_SHELF.run();
    Serial.println("shelfupButtonCallback");
  }
}


void shelfdownButtonCallback()
{
  // move until home limit or release
  if (cycle_not_running)
  {
    shelfdownButton.read();
    PRODUCT_SHELF.stop();
    PRODUCT_SHELF.run();
    Serial.println("shelfdownButtonCallback");
  }
}



void  shelf_limitInterupt()
{
  noInterrupts();
  if (digitalRead(PRODUCT_SHELF_LIMIT_PIN))
  {
    // stop moving send stop then run the stop
   // PRODUCT_SHELF.stop();
   // PRODUCT_SHELF.run();
    product_shelf_limit = true;

  }
  else
  {
    product_shelf_limit = false;
  }
  interrupts();
}


void shelf_homeInterupt()
{
  noInterrupts();
  if (digitalRead(PRODUCT_SHELF_HOME_PIN))

  {
    product_shelf_home = true;
  }
  else
  {
    product_shelf_home = false;
  }
  interrupts();
}

void table_limitInterupt()
{
  noInterrupts();
  if (digitalRead(TABLE_BAR_LIMIT_PIN))
  {
    table_bar_limit = true;
  }
  else
  {
    table_bar_limit = false;
  }
  interrupts();
}


void  table_homeInterupt()
{
  noInterrupts();
  if (digitalRead(TABLE_BAR_HOME_PIN))
  {
    table_bar_home = true;
  }
  else
  {
    table_bar_home = false;
  }
  interrupts();
}





void startButtonCallback()
{
  if (digitalRead(OPERATOR_START_SWITCH_PIN) )
  {
     if (cycle_not_running )
     {
     test_counter++;
     }
  }
  else
  {
    //
  }
}



ISR (PCINT0_vect) // handle pin change interrupts
{
  pciDisable();

  if (digitalRead(OPERATOR_EMERGENCY_STOP_INPUT_PIN))
  {
    // with systems - the interupt stops coms with LCD
    digitalWrite(EMERGENCY_STOP_LED, HIGH);
    emergency_stop = true;
    TABLE_BAR.stop();
    PRODUCT_SHELF.stop();
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("EMERGENCY STOP     ");
    LCD.setCursor(0, 1);
    LCD.print("Power off tto reset ");
  }

  // just read the stock in place sensor as it changed
  stock_in_place = !digitalRead(PRODUCT_IN_PLACE_INPUT_PIN);   // HIGH for no so invert it

  pciEnable();
}

void EMERGENCY_STOP_PROCESS()
{
  pciDisable();
  digitalWrite(EMERGENCY_STOP_LED, HIGH);
  emergency_stop = true;
  TABLE_BAR.stop();
  PRODUCT_SHELF.stop();
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("EMERGENCY STOP     ");
  LCD.setCursor(0, 1);
  LCD.print("Power off to reset ");
  while (1);
}


void startup()
{
  /* on start, we require these conditions

     product_in_place switch must be HIGH - no items

     product_shelf_HOME should be on
     product_shelf_LIMIT should be off

     TABLE_BAR_LIMIT_PIN should be off
     TABLE_BAR_HOME_PIN should be on
  */


  debug_switches("Startup Called");


  // checks for impossible
  if (table_bar_home &&  table_bar_limit)
  {
    debug_switches("HOME AND LIMIT ON TABLE");

    // if the home positions are not home after homing then something is wrong
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print(" EMERGENCY STOP     ");
    LCD.setCursor(0, 1);
    LCD.print("T HOME-LIMIT ON    ");
    EMERGENCY_STOP_PROCESS();
  }

  if (product_shelf_home &&  product_shelf_limit)
  {
    debug_switches("HOME AND LIMIT ON SHELF");

    // if the home positions are not home after homing then something is wrong
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print(" EMERGENCY STOP     ");
    LCD.setCursor(0, 1);
    LCD.print("S HOME-LIMIT ON    ");
    EMERGENCY_STOP_PROCESS();
  }


  if (!table_bar_home || !product_shelf_home)   // the steppers are not in home position
  {
    debug_switches("Steppers not in home position");
    if (!table_bar_home) move_table_bar_home_on_startup();
    if (!product_shelf_home)  move_shelf_home_on_startup();
    
  }

  
  debug_switches("  AFTER HOMING  ");

  if (!table_bar_home && !product_shelf_home)
  {
    debug_switches("Steppers STILL not in home position after home");

    // if the home positions are not home after homing then something is wrong
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print(" EMERGENCY STOP     ");
    LCD.setCursor(0, 1);
    LCD.print("ERROR CANT HOME    ");
    millisdelay(2000, false);
    EMERGENCY_STOP_PROCESS();
  }

  // getting here means the homing worked
  // now check the prox sensor - if we are home,
  // then it should not have any stock there

  if (stock_in_place )
  {
    debug_switches("INCORRECT stock in place");
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print(" EMERGENCY STOP     ");
    LCD.setCursor(0, 1);
    LCD.print("CLEAR ITEMS     ");
    EMERGENCY_STOP_PROCESS();
  }

  //  get here and the system should be in home positions, calibrated and no stock near sensor

  // test the operator to push start after loading stock
}


void move_table_bar_home_on_startup()
{
  initial_present_position = -10000;  // reverse
  table_bar_home  = digitalRead(TABLE_BAR_HOME_PIN);
  TABLE_BAR.moveTo(initial_present_position);

  TABLE_BAR.setMaxSpeed(1000);
  TABLE_BAR.setSpeed(500);
  TABLE_BAR.setAcceleration(1000);


  while (!table_bar_home && !emergency_stop) {  // Make the Stepper move  until the switch is activated
    buttonrefresh();

    LCD.setCursor(11, 1);
    LCD.print(TABLE_BAR.distanceToGo());
    TABLE_BAR.enableOutputs();
    TABLE_BAR.runSpeedToPosition();
  }
  TABLE_BAR.setCurrentPosition(table_stepper_home_position); // at home - set position to zero
  table_bar_home_calibrated = true;
  debug_switches("move_table_bar_home_on_startup");
}






void move_shelf_home_on_startup()
{
  initial_present_position = -10000;  // down
  product_shelf_home = digitalRead(PRODUCT_SHELF_HOME_PIN);
  PRODUCT_SHELF.moveTo(initial_present_position);

  PRODUCT_SHELF.setMaxSpeed(1000);
  PRODUCT_SHELF.setSpeed(500);
  PRODUCT_SHELF.setAcceleration(1000);

  while (!product_shelf_home && !emergency_stop )
  {
    PRODUCT_SHELF.run();  //run
    buttonrefresh();

    LCD.setCursor(11, 1);
    LCD.print(PRODUCT_SHELF.distanceToGo());
    PRODUCT_SHELF.enableOutputs();
    PRODUCT_SHELF.runSpeedToPosition();
  }
  PRODUCT_SHELF.setCurrentPosition(shelf_stepper_home_position); // at home - set position to zero
  product_shelf_home_calibrated = true;
 
  debug_switches("move shelf home on startup");

}





void move_table_bar_home()
{
  table_bar_home  = digitalRead(TABLE_BAR_HOME_PIN);
  // if we have homed the table bar - then we know where table_stepper_home_position is
  buttonrefresh();

  if (table_bar_home_calibrated )
  {
    TABLE_BAR.setMaxSpeed(1000);
    TABLE_BAR.setSpeed(1000);
    TABLE_BAR.setAcceleration(1000);


    TABLE_BAR.moveTo(table_stepper_home_position);
    while (!table_bar_home && !emergency_stop )
    {
      TABLE_BAR.enableOutputs();
      TABLE_BAR.runSpeedToPosition();

      LCD.setCursor(11, 1);
      LCD.print(TABLE_BAR.distanceToGo());
    }
  }
  else
  {
    // if its not claibrated run calibration
    move_table_bar_home_on_startup();
  }


  debug_switches("move_table_bar_home");
}


void move_table_bar_forward()
{

  // we should NOT move the bar if the product shelf is not home
  if (!product_shelf_home)
  {
    // have to move product shelf down out of the way
    move_product_shelf_home();

  }
 
    
    TABLE_BAR.moveTo(10000);  // Set the position to move to - anywhere forward
    while (!stock_in_place && !table_bar_limit && !emergency_stop ) {
      // Make the Stepper move  until the switch is activated (LOW)
      // stop if we reach table_bar limit with no stock
   
      LCD.setCursor(11, 1);
      LCD.print(TABLE_BAR.currentPosition());
      TABLE_BAR.enableOutputs();
      TABLE_BAR.runSpeedToPosition();
    
  }

  debug_switches("move_table_bar_forward");
}





void move_product_shelf_home()
{
  product_shelf_home = digitalRead(PRODUCT_SHELF_HOME_PIN);
  // if we have homes the product shelf - then we know where 0 is

  if (product_shelf_home_calibrated )
  {

    PRODUCT_SHELF.moveTo(shelf_stepper_home_position);
    while (!product_shelf_home && !emergency_stop )
    {
      PRODUCT_SHELF.enableOutputs();
      PRODUCT_SHELF.runSpeedToPosition();
      LCD.setCursor(11, 1);
      LCD.print(PRODUCT_SHELF.currentPosition());
    }
  }
  else
  {
    // run startup calibration
    move_shelf_home_on_startup();
  }
  debug_switches("move_product_shelf_home");

}

void move_product_shelf_up()
{
  PRODUCT_SHELF.moveTo(10000);  // Set the position to move to anywhere HIGH
  product_shelf_limit = digitalRead(PRODUCT_SHELF_LIMIT_PIN);

  while (!product_shelf_limit && !emergency_stop) {  // Make the Stepper move  until the switch is activated
    {

      LCD.setCursor(11, 1);
      LCD.print(PRODUCT_SHELF.currentPosition());
      PRODUCT_SHELF.enableOutputs();
      PRODUCT_SHELF.runSpeedToPosition();
    }
    debug_switches("move_product_shelf_up");
  }


}


void millisdelay(long delaywanted, bool showlcd)
{
  long time_now = millis();
  int flippercount = 0;
  while (millis() - time_now < delaywanted )
  {
    // incase something happened while we are waiting
    if (showlcd)
    {

      if ( millis() % 100 == 0)
      {
        flippercount++;
        if (flippercount > 4) flippercount = 0;
        LCD.setCursor(15, 1);
        LCD.print(flipper[flippercount]);
        LCD.setCursor(15, 1);
      }

    }



    if (emergency_stop)
    {
      delaywanted = 0;
      EMERGENCY_STOP_PROCESS();
    }

  }
  // clear the last flip if required
  if (showlcd)  LCD.print(" ");
}

void debug_switches(String notes)
{
  String notice = "";

  change_check =
                  product_shelf_home_calibrated  * 1000000
                 + table_bar_home_calibrated  * 1000000
                 + stock_in_place * 100000
                 + product_shelf_home  * 10000
                 + product_shelf_limit * 1000
                 + table_bar_home * 100
                 + table_bar_limit * 10
                 + emergency_stop;


  if ( change_check != previous_change_check)
  {
    if ( debug_heading_counter % 5 == 0)
    { DEBUG_SERIAL.println( "-----------------------------------------------------------------------------------");
      DEBUG_SERIAL.println( "STOCK | SHELF     | TABLE     |EMERG | SHELF\t\t | TABLE \t|   ");
      DEBUG_SERIAL.println( "THERE | H  L  CAL | H  L  CAL |STOP  | H\tP\t |  H\t  P\t|  ");
      DEBUG_SERIAL.println( "-------------------------------------------------------------------------------------");
    }
    debug_heading_counter++;


    (stock_in_place ? notice = " yes  |" : notice = "  no  |"   );
    DEBUG_SERIAL.print(notice);

    (product_shelf_home ? notice = " 1" :  notice = " 0");
    DEBUG_SERIAL.print(notice);

    (product_shelf_limit ? notice = "  1 " :  notice = "  0 ");
    DEBUG_SERIAL.print(notice);
    (product_shelf_home_calibrated ? notice = "  1  |" :  notice = "  0  |");
    DEBUG_SERIAL.print(notice);


    (table_bar_home ? notice = " 1" :  notice = " 0 ");
    DEBUG_SERIAL.print(notice);

    (table_bar_limit ? notice = " 1" :  notice = "  0");
    DEBUG_SERIAL.print(notice);
    (table_bar_home_calibrated ? notice = "  1  |" :  notice = "  0  |");
    DEBUG_SERIAL.print(notice);
    (emergency_stop ? notice = "\tyes  |" :  notice = "\tno   |");
    DEBUG_SERIAL.print(notice);
    DEBUG_SERIAL.print("  ");


    DEBUG_SERIAL.print(shelf_stepper_home_position);
    DEBUG_SERIAL.print("\t");
  //  DEBUG_SERIAL.print( shelf_stepper_limit_position);
  //  DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(PRODUCT_SHELF.currentPosition());
    DEBUG_SERIAL.print("\t |  ");


    DEBUG_SERIAL.print(table_stepper_home_position);
    DEBUG_SERIAL.print("\t  ");
   // DEBUG_SERIAL.print( table_stepper_limit_position);
  //  DEBUG_SERIAL.print("\t");
    DEBUG_SERIAL.print(TABLE_BAR.currentPosition());
    DEBUG_SERIAL.print("\t  T=");
    DEBUG_SERIAL.print(String(test_counter));
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(notes);
    DEBUG_SERIAL.println();
    previous_change_check = change_check;
  }
}

void displayLCD(int linenumber, String thecomment)
{
  LCD.setCursor(0, linenumber);
  LCD.print("                    ");
  LCD.setCursor(0, linenumber);
  LCD.print(thecomment);
}
