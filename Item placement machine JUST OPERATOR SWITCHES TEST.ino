//https://wokwi.com/projects/348074752483525202

#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>


LiquidCrystal_I2C LCD = LiquidCrystal_I2C(0x27, 16, 2);

#define DEBUG true  //set to true for debug output, false for no debug output
#define DEBUG_SERIAL if(DEBUG)Serial
#define DEBUG_PAUSE if(DEBUG)millisdelay
#define DEBUG_LCD if(DEBUG)displayLCD

volatile int debug_heading_counter = 0;
volatile int change_check;
volatile int previous_change_check;
volatile int thechange;



// MACHINE switch inputs
#define PRODUCT_IN_PLACE_INPUT_PIN 2
#define TABLE_BAR_LIMIT_PIN 19
#define TABLE_BAR_HOME_PIN 11
#define PRODUCT_SHELF_HOME_PIN 10
#define PRODUCT_SHELF_LIMIT_PIN 18

// OPERATOR SWITCHES
#define OPERATOR_START_SWITCH_PIN  53
#define OPERATOR_HOME_SWITCH_PIN  51
#define OPERATOR_EMERGENCY_STOP_INPUT_PIN 3


// Vertical Stepper
#define PRODUCT_SHELF_STEPPER_CONT_EN 23
#define PRODUCT_SHELF_STEPPER_CONT_STEP 27
#define PRODUCT_SHELF_STEPPER_CONT_DIR 25


// Horizonal Stepper
#define TABLE_BAR_STEPPER_CONT_EN 29
#define TABLE_BAR_STEPPER_CONT_STEP 31
#define TABLE_BAR_STEPPER_CONT_DIR 33

// Misc  outputs
#define  EMERGENCY_STOP_LED 12

// Stepper constants
const int stepsPerRevolution = 200 ;              // set for steppers
int initial_present_position;                     // general position variable
int stepper_delay = 50  ;                          // ms for move of 1


// Boolean flags for status product_in_place sensor
volatile boolean stock_in_place                    = false;
volatile boolean previous_stock_in_place           = true;

// Boolean flags for status product_shelf
volatile boolean product_shelf_home_calibrated     = false;
volatile boolean product_shelf_home                = false;
volatile boolean product_shelf_limit               = false;
volatile boolean previous_product_shelf_home       = true;
volatile boolean previous_product_shelf_limit      = true;

// Boolean flags for status table_bar
volatile boolean table_bar_home_calibrated         = false;
volatile boolean table_bar_home                    = false;
volatile boolean table_bar_limit                   = false;
volatile boolean previous_table_bar_limit          = true;
volatile boolean previous_table_bar_home           = true;

// operator buttons
volatile boolean emergency_stop                    = false;
volatile boolean start_pushed                      = false;
volatile boolean home_pushed                       = false;
volatile boolean previous_start_pushed             = true;
volatile boolean previous_home_pushed              = false;

// operator  buttons debounce
long debounceTime = 30;

// stepper position variables
volatile int shelf_stepper_position = 0  ;                 // shelf position
volatile int horizontal_stepper_position = 0  ;            //  horizontal position




//testing variables and leds
volatile boolean completed_tests = false;
int test_counter = 0;
int test_counter_previous = 0;
int logical_table_bar_HOME_led = 41;
int logical_table_bar_LIMIT_led = 43;
int logical_product_shelf_HOME_led = 45;
int logical_product_shelf_LIMIT_led = 47;

volatile unsigned long StateChangeMillis = 0;




void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


// Setup steppers
AccelStepper TABLE_BAR(1, TABLE_BAR_STEPPER_CONT_STEP, TABLE_BAR_STEPPER_CONT_DIR);
AccelStepper PRODUCT_SHELF(1, PRODUCT_SHELF_STEPPER_CONT_STEP, PRODUCT_SHELF_STEPPER_CONT_DIR);



void setup() {

  Serial.begin(9600);
  // testing only
  //startButton.begin();

  // Stepper Horizonal
  TABLE_BAR.setEnablePin(TABLE_BAR_STEPPER_CONT_EN);
  TABLE_BAR.setPinsInverted(false, false, true);
  TABLE_BAR.enableOutputs();
  TABLE_BAR.setMaxSpeed(200);
  TABLE_BAR.setSpeed(20);

  // Stepper Vertical
  PRODUCT_SHELF.setEnablePin(PRODUCT_SHELF_STEPPER_CONT_EN);
  PRODUCT_SHELF.setPinsInverted(false, false, true);
  PRODUCT_SHELF.enableOutputs();
  PRODUCT_SHELF.setMaxSpeed(200);
  PRODUCT_SHELF.setSpeed(20);


  // set pin modes - switches have pull down resistors - product_in_place switch active HIGH/LOW
  pinMode(PRODUCT_IN_PLACE_INPUT_PIN, INPUT);     // High normally - low on switched -
  // we use inverted value to get logic levels right way around

  pinMode(TABLE_BAR_LIMIT_PIN, INPUT);           // low normally - high on switched
  pinMode(TABLE_BAR_HOME_PIN, INPUT);            // low normally - high on switched

  pinMode(PRODUCT_SHELF_HOME_PIN, INPUT);         // low normally - high on switched
  pinMode(PRODUCT_SHELF_LIMIT_PIN, INPUT);        // low normally - high on switched

  pinMode(OPERATOR_START_SWITCH_PIN, INPUT);      // low normally - high on switched
  pinMode(OPERATOR_HOME_SWITCH_PIN, INPUT);       // low normally - high on switched

  pinMode(EMERGENCY_STOP_LED, OUTPUT);        // LED or LIGHT for emergency stop condition

  // pin mode on the stepper enables - most likely not required
  pinMode(TABLE_BAR_STEPPER_CONT_EN, OUTPUT);
  pinMode(PRODUCT_SHELF_STEPPER_CONT_EN, OUTPUT);


  // TESTING LOGIC
  pinMode(logical_table_bar_HOME_led, OUTPUT);
  pinMode(logical_table_bar_LIMIT_led, OUTPUT);
  pinMode(logical_product_shelf_HOME_led, OUTPUT);
  pinMode(logical_product_shelf_LIMIT_led, OUTPUT);

  /* -------------------------------------------
    real startup sequence for safety
    -------------------------------------------
  */

  // ENSURE STEPPERS ARE NOT RUNNING
  TABLE_BAR.stop();
  PRODUCT_SHELF.stop();


  // START THE SCREEN
  LCD.init();
  LCD.backlight();
  LCD.setCursor(0, 0);

  // SET THE SWITCH HARDWARE INTERUPTS
  attachInterrupt(digitalPinToInterrupt(PRODUCT_IN_PLACE_INPUT_PIN), proxInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OPERATOR_EMERGENCY_STOP_INPUT_PIN), EMERGENCY_STOP_Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(PRODUCT_SHELF_LIMIT_PIN), SHELF_LIMIT_Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TABLE_BAR_LIMIT_PIN), TABLE_BAR_LIMIT_Interrupt, CHANGE);

  // Attach the PinChangeInterrupts
  pciEnable();


  LCD.setCursor(0, 0);
  LCD.print("START HOME EMERG");


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


  pciSetup(TABLE_BAR_HOME_PIN);
  pciSetup(PRODUCT_SHELF_HOME_PIN);
  pciSetup(OPERATOR_START_SWITCH_PIN);
  pciSetup(OPERATOR_HOME_SWITCH_PIN);
}

void loop() {


  String notice = "  n   ";
  if (start_pushed  )
  {
    notice = "  y   ";
    start_pushed = false;
  }
  LCD.setCursor(0, 1);
  LCD.print( notice);


  notice = "n " ;
  if (home_pushed  )
  {
    notice = "y ";
    home_pushed = false;

  }
  LCD.setCursor(8, 1);
  LCD.print( notice);


  notice = "n ";
  if (emergency_stop  )
  {
    notice = "y" ;
    emergency_stop = false;

  }
  LCD.setCursor(14, 1);
  LCD.print( notice);
  delay(1000);
}

ISR (PCINT0_vect) // handle pin change interrupts
{
  // turn off interupts here
  pciDisable();

  //Read the pins as they are now we have 4
  bool STARTpinState = digitalRead(OPERATOR_START_SWITCH_PIN);
  bool HOMEpinState = digitalRead(OPERATOR_HOME_SWITCH_PIN);
  bool PRODUCT_SHELF_HOMEpinState = digitalRead(PRODUCT_SHELF_HOME_PIN);
  bool TABLE_BAR_HOMEpinState = digitalRead(TABLE_BAR_HOME_PIN);

  if (millis() - StateChangeMillis > debounceTime )
  {
    // START BUTTON
    if (STARTpinState) // HIGH
    {
      start_pushed = true;
    }
    // HOME BUTTON
    if (HOMEpinState)
    {
      home_pushed = true;
    }

    // SHELF_HOME_Interrupt()
    if (PRODUCT_SHELF_HOMEpinState )
    {
      // stop moving
      PRODUCT_SHELF.stop();
      shelf_stepper_position = PRODUCT_SHELF.currentPosition();
      product_shelf_home = true;
    }
    else
    {
      product_shelf_home = false;
    }

    //TABLE_BAR_HOME_PIN_Interrupt()
    if (TABLE_BAR_HOMEpinState )
    {
      //stop moving
      TABLE_BAR.stop();
      horizontal_stepper_position = TABLE_BAR.currentPosition() ;
      table_bar_home = true;
    }
    else
    {
      table_bar_home = false;
    }
    //last time we were here
    StateChangeMillis = millis();
    // enable interupts again
  }
  pciEnable();
}

void SHELF_LIMIT_Interrupt()
{
  // stop moving
  PRODUCT_SHELF.stop();
  shelf_stepper_position = PRODUCT_SHELF.currentPosition();
  product_shelf_limit = digitalRead(PRODUCT_SHELF_LIMIT_PIN);
}

void TABLE_BAR_LIMIT_Interrupt()
{
  //stop moving
  TABLE_BAR.stop();
  horizontal_stepper_position = TABLE_BAR.currentPosition() ;
  table_bar_limit = digitalRead(TABLE_BAR_LIMIT_PIN);
}

void EMERGENCY_STOP_Interrupt()
{
  // with systems - the interupt stops coms with LCD
  digitalWrite(EMERGENCY_STOP_LED, HIGH);
  emergency_stop = true;
  TABLE_BAR.stop();
  PRODUCT_SHELF.stop();


}

void proxInterrupt() {
  stock_in_place = !digitalRead(PRODUCT_IN_PLACE_INPUT_PIN);   // HIGH for no so invert it
}


void millisdelay(long delaywanted)
{
  long time_now = millis();
  while (millis() - time_now < delaywanted )
  {
    // incase something happened while we are waiting
    if (emergency_stop)
    {
      delaywanted = 0;
      EMERGENCY_STOP_Interrupt();
    }
  }
}

void debug_switches(String notes)
{


  String notice = "";



  change_check = stock_in_place * 100000
                 + product_shelf_home  * 10000
                 + product_shelf_limit * 1000
                 + table_bar_home * 100
                 + table_bar_limit * 10
                 + emergency_stop;


  if ( change_check != previous_change_check)
  {
    if ( debug_heading_counter % 5 == 0)
    { DEBUG_SERIAL.println( "-----------------------------------------------------------------------------------");
      DEBUG_SERIAL.println( "STOCK |   SHELF            |     TABLE             | EMERG |");
      DEBUG_SERIAL.println( "THERE | HOME   LIMIT CALIB |   HOME   LIMIT  CALIB | STOP  |     ");
      DEBUG_SERIAL.println( "-------------------------------------------------------------------------------------");
    }
    debug_heading_counter++;
    (stock_in_place ? notice = " yes  |" : notice = "  no  |"   );
    DEBUG_SERIAL.print(notice);
    (product_shelf_home ? notice = "\tyes" :  notice = "\tno ");
    DEBUG_SERIAL.print(notice);
    (product_shelf_home_calibrated ? notice = "\tyes" :  notice = "\tno ");
    DEBUG_SERIAL.print(notice);
    (product_shelf_limit ? notice = "\tyes|" :  notice = "\tno |");
    DEBUG_SERIAL.print(notice);
    (table_bar_home ? notice = "\tyes" :  notice = "\tno ");
    DEBUG_SERIAL.print(notice);
    (table_bar_home_calibrated ? notice = "\tyes" :  notice = "\tno ");
    DEBUG_SERIAL.print(notice);
    (table_bar_limit ? notice = "\tyes|" :  notice = "\tno |");
    DEBUG_SERIAL.print(notice);
    (emergency_stop ? notice = "\tyes|" :  notice = "\tno |");
    DEBUG_SERIAL.print(notice);
    DEBUG_SERIAL.print("  ");
    DEBUG_SERIAL.print(notes);
    DEBUG_SERIAL.print(" SHELF=");
    DEBUG_SERIAL.print(PRODUCT_SHELF.currentPosition());
    DEBUG_SERIAL.print(" TABLE=");
    DEBUG_SERIAL.print(TABLE_BAR.currentPosition());
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
