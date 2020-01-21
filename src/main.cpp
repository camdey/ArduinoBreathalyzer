/*
          \\\\\\\\Breathalyzer with MQ-3 Sensor and ArduinoUno R3////////

          10kohm potentiometer for calibrating warm-up time and initiating functions
          LED light for visual signalling
          Progress bars to show represent time during several of the functions (WarmUp, BreathSample, etc)

  * Starts with a sensor warm-up with warm-up time determined by pot. wiper placement (voltage divider).

  * Next it goes to a calibration mode where it takes a 10-reading moving average of the sensor and calculates
     the variance of the 10-reading sample. If the variance drops below 0.20, indicating sensor stability, calibration
     finishes and moves on. Override for calibration by pressing button.

  * Raw analog sensor value should always be constant in the same atmospheric conditions (e.g. 20*C, 65% RH).
     For clean air this fixed at 17 based on tests I performed over several hours. Breathing on the sensor increases
     the analog value by about 30 due to increased humidity and temperature (~35*C, 95% RH). So both RS for clean air
     and R0 (defined by 0.4mg/L ethanol in the spec sheet) are held constant. R0 works out as 9.87kohms.

  * During the loop() phase, when the button is depressed the user can blow on the sensor to take a breath sample.
     The highest value during this sampling period is recorded and used to determine mg/L, PPM, and BAC. The analog value
     (sensorMax) is deducted by 30 to account for the increased temp+humidity of human breath. This value (sensorMax_Adj)
     is then used to determine the aforementioned measures.

  * Standard drinks available is calculated based on the body weight of the individual (set to 96kg), gender (0.7 for males)
     and the amount of alcohol already consumed. Based on Widmark formula but doesn't include the time variable at this stage.

  * If the pot. is turned all the way clockwise then diagnostic stats will display on the screen, this is to ensure
     that the readings and sampling period have recorded reasonable values. If the pot is turned all the way anti-clockwise
     then the calculated values are reset to 0 for the next test.

  Arduino Breathalyzer Project v0.91
  Created 14 April 2016

*/

#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//-----sensor variables-----//
float MQ3 = A0; // sensor pin
float sensorValue; // raw ADC value
float sensorValue_air; // raw ADC value during calibration
float sensorValue_alc; // raw ADC value during breath test
float sensor_volt_alc; // V-OUT calculated during breath test
float sensorMax = 0; // store Max raw ADC value
float sensorMaxCal = 0; // max raw ADC value during calibration
float sensorMax_adj = 0; // temp-humidity adjusted value

//-----component variables-----//
const int ButtonPin = 2; // digital pin #2 - button pin
const float LED = A1; // analog pin #1 - LED pin
const int pot = A2; // analog pin #2 - potentiometer pin
volatile int ledState = LOW; // initialise ledState for blink at off
volatile int ledCount; // count variable for LED blink
int potValue = 0; // raw ADC value of pot.
float potMap = 0; // inverse ADC of pot. converted to seconds for delay
float potDelay; // potMap on 0-16 scale, constant for delay timer

//-----resistance variables-----//
float RS_air; //  Get the value of RS via clear air
float RS_alc; //  Get the value of RS_alc via breath test
float R0 = 9.87; // R0 to use for breath test
float RS_R0 = 0.0; // Get the RS_alc/R0 ratio value for linear regression model

//-----Misc variables-----//
volatile int blockCursor3 = 0;
volatile int sampleProgress = 1;
int diagLoop = 0; // variable for looping second part of Diagnostics() after setup()

//-----conversion variables-----//
float mg_L = 0; // variable for mg/L
float ppm = 0; // variable for ppm
float bac = 0; // variable for bac

//-----calibration variables-----//
float array[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // calibration array
float avArray = 0; // variable for calculating calibration average
float varArray = 0; // variable for calculating calibration variance
int expArray = 2; // variable for exponent in variance calculation
int y = 0; // always runs if statement after array has been filled for the first time

//-----std drinks variables-----//
float body_const = 67.2; // 96kg*0.7 where 0.7 = coefficient for men
float alc_consumed = 0; // grams of alcohol consumed based on BAC
float bac_grams = 33.6; // grams of alcohol to reach 0.05 BAC limit, 3.3 std drinks
float alc_available = 0; // bac_grams minus alc_consumed
float std_drinks = 0; //# of standard drinks until limit reached


//-----setup - runs once-----//
/* WarmUp(), Calibrate(), and Diagnostics() run during setup
 * Button, LED, and serial initialised
 */

void setup() {
  lcd.clear();
  Serial.begin(9600);  // initialize serial communications at 9600 bps
  pinMode(ButtonPin, INPUT_PULLUP); // pull-up resistor, button reads HIGH when open, LOW when pressed
  pinMode(LED, OUTPUT); // set LED pin mode
  lcd.begin(16, 2); // LCD dimensions (16x2)

  WarmUp(); // run warmup loop

  lcd.clear();
  lcd.setCursor (0, 0);
  lcd.print("Calibrating...");

  Calibrate(); //run calibration loop

  Diagnostics(); // run Diagnostics loop

}

//-----loop - runs continuously-----//
/* Will show "Hold button to begin" until button is pressed
 * When button is pressed, breath sampling commences
 * Amount of standard drinks left calculated based on BAC reading
 * Diagnostics() can be looped on screen if potMap > 59
 * Sensor measures reset to 0 if potMap < 1
 */

void loop() {
  int bp = 0;   // boolean variable for running BreathSample(), changes to 1 when button pressed
  diagLoop = 1; // enables second part of Diagnostics() if run in loop()

  // clear BreathSample() progress
  blockCursor3 = 0; // cursor block for BreathSample() progress bar
  sampleProgress = 1; // initialise at 1 for BreathSample() progress bar

  // if button pressed, run BreathSample()
  if (digitalRead(ButtonPin) == LOW) {
    bp = 1;
  }

  if (bp == 1) {
    BreathSample();
    lcd.clear();
    bp = 0; // reset bp to 0 to prevent auto-looping

    // turn on LED if BAC 0.05 exceeded //
    if (bac >= 0.05) {
      digitalWrite(LED, HIGH);
    } else if {
      digitalWrite(LED, LOW);
    }
    // print BAC value and Result //
    lcd.setCursor(0, 0);
    lcd.print("BAC = ");
    lcd.setCursor(6, 0);
    lcd.print(bac, 3);
    lcd.setCursor(11, 0);
    lcd.print("%");
    lcd.setCursor(0, 1);
    if (bac < 0.021) {
      lcd.print("N/A");
    } else if (bac < 0.03) {
      lcd.print("No worries");
    } else if (bac < 0.04) {
      lcd.print("Last Call");
    } else if (bac < 0.049) {
      lcd.print("Sleep it off");
    } else if (bac < 0.079) {
      lcd.print("Fail - Drunk");
    } else if (bac < 0.08) {
      lcd.print("CriminalOfffence");
    } else if (bac < 0.1) {
      lcd.print("Sloshed");
    } else if (bac < 0.2) {
      lcd.print("Shitfaced");
    } else if (bac < 0.3) {
      lcd.print("Death awaits");
    } else {
      lcd.print("Dead");
    }
    delay(1000 * 6);

    // determine std. drinks available //
    alc_consumed = (body_const * (bac * 10));
    alc_available = (bac_grams - alc_consumed);
    std_drinks = (alc_available / 10);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You have");
    lcd.setCursor(9, 0);
    lcd.print(std_drinks, 1);
    lcd.setCursor(0, 1);
    lcd.print("std. drinks left");
    delay(1000 * 6);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Wait 10sec...");

    // progress bar for 10sec wait, 10/16 = 0.625ms loop delay per block
    int blockCursor4 = 0;
    for (int z = 0; z < 16; z ++) {
      lcd.setCursor(blockCursor4, 1);
      lcd.write(1023); // 1023 = block character
      blockCursor4 = blockCursor4 + 1;
      delay(625);
    }

  } else {
    lcd.setCursor(2, 0);
    lcd.print(" Hold button");
    lcd.setCursor(4, 1);
    lcd.print("to begin");
    delay(200);
  }

  // show just the Diagnostics() function if pot value above 60)
  potValue = analogRead(pot); // read analog value from pot
  potMap = ((1023 - potValue) / 17.05); // convert analog value (0-1023) to 0-60
  if (potMap > 59) {
    Diagnostics();
  }
  lcd.clear();

  // reset values if pot value above 60)
  if (potMap < 1) {
    sensorMax = 0;
    sensorMax_adj = 0;
    ppm = 0;
    mg_L = 0;
    bac = 0;
    RS_R0 = 0;
    std_drinks = 0;

    lcd.setCursor(0, 0);
    lcd.print("Resetting...");
    delay(1000 * 4);
    lcd.clear();
  }
}

//-----loop interrupt - button press-----//
void BreathSample() {

  /*  blinks LED on/off every 1 second, resets after count > 40
   *   SampleProgress (1-6), adds block character to progress bar when = 6
   *   blockCursor3 (0-16), cursor position for adding block character
   *   Run for loop if sampleProgress and blockCursor3 are reset while button is pressed
   *   75ms delay, sensor samples 13 times a second.
   *   16 characters on LCD, character added every 6th loop = 96 loops
   *   Max loop time is 7.2sec (96*75)
   */

  sensorMax = 0; // clear max value set during calibration
  if (sampleProgress == 1 && blockCursor3 == 0 && digitalRead(ButtonPin) == LOW) {

    lcd.clear();

    for (int s = 0; s < 100; s++) {

      delay(75); // 16*6 = 96, 96*75ms = 7.2sec
      if (digitalRead(ButtonPin) == HIGH) {
        s = 100;
      }

      // led blink //
      ledCount = (ledCount + 1); // ++ 1 every 75ms (13/sec)
      if (ledCount >= 20) { // limit of 20
        ledCount = 0; // reset to 0
      }
      if (ledCount <= 10) { //if less then 20, ledState = high, else low
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(LED, ledState); // write ledState

      lcd.setCursor(0, 0);
      lcd.print("Blow...");
      lcd.setCursor(7, 0);
      lcd.print(sensorMax_adj); // print live feed of ADC value adjusted for temp-humidity

      // progress bar code, add block every 6th loop then reset back to 1 if = 6
      if (sampleProgress >= 6) {
        sampleProgress = 1;
        lcd.setCursor(blockCursor3, 1);
        lcd.write(1023);
        blockCursor3 = blockCursor3 + 1;
      } else {
        sampleProgress = sampleProgress + 1;
      }

      // if progress bar complete, reset to 0 and reset sampleProgress to 1
      if (blockCursor3 > 16) {
        sampleProgress = 1;
        blockCursor3 = 0;
        lcd.clear();
      }

      // read sensor, set max reading as the RS value used for calculating BAC
      sensorValue_alc = analogRead(MQ3);
      if (sensorValue_alc > sensorMax) {
        sensorMax = sensorValue_alc;
      }

      // convert readings to breathalyser variables
      int htAdj = 30; // temp-humidity adjustment constant
      sensorMax_adj = (sensorMax - htAdj); // adjusts breath sample reading by humidity+temp effect
      sensor_volt_alc = sensorMax_adj / 1024 * 5.0; // calculate voltage from analog value
      RS_alc = (5.0 / sensor_volt_alc - 1) * 10.0; // calculate max resistance of sensor from breath test
      RS_R0 = RS_alc / R0; // The ratio of sensor resistance to R0 (0.4mg/L alc)
      float RS_R0_ln = 0; // variable for natural log of RS_R0 variable
      RS_R0_ln = log(RS_R0); // natural log of RS_R0 for regression formula

      // convert sensor measures to measures of alcohol content
      mg_L = exp(-0.884301169 + (-1.489113359 * RS_R0_ln)); // log-transformed linear regression formula from 25 data points
      ppm = (mg_L / 0.4) * 220; // mg/L to PPM formula
      bac = (((mg_L / 1000) * 2100) * 0.1); // mg/L to BAC formula
      if (bac < 0.021) { // if BAC less than 0.021, then BAC = 0 (lowest mg/L reading from graph = 0.1mg/L)
        bac = 0.0;
      }

    }
  } else {
    sampleProgress = 1;
    blockCursor3 = 0;
    ledCount = 0;
  }           // else

  digitalWrite(LED, LOW); // turn off LED
}

//-----Calibration measures - shows when potMap > 59-----//
void Diagnostics() {

  /* Runs through various diagnostic/calibration measures
   *  ADC Cur and Cal. Max run during setup, other measures run if potMap > 59
   *  Includes: current value, max calibration value, mg/L, PPM, RS/R) and max sample value
   */

  // print current ADC and Cal Max //
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ADC Cur. = ");
  lcd.setCursor(11, 0);
  lcd.print(analogRead(MQ3));
  lcd.setCursor(0, 1);
  lcd.print("Cal. Max = ");
  lcd.setCursor(11, 1);
  lcd.print(sensorMaxCal, 0);
  delay(1000 * 5);
  lcd.clear();

  // prevents code running during setup as values not determined until sample
  if (diagLoop == 1) {

    // print mg/L and PPM //
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("mg/L = ");
    lcd.setCursor(7, 0);
    lcd.print(mg_L, 3);
    lcd.setCursor(0, 1);
    lcd.print("PPM = ");
    lcd.setCursor(6, 1);
    lcd.print(ppm, 0);
    delay(1000 * 5);

    // print RS/RO and Max ADC //
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RS/R0 = ");
    lcd.setCursor(8, 0);
    lcd.print(RS_R0);
    lcd.setCursor(0, 1);
    lcd.print("Max ADC = ");
    lcd.setCursor(10, 1);
    lcd.print(sensorMax_adj);
    delay(1000 * 5);
    lcd.clear();

  }

}


//-----Calibration loop-----//
void Calibrate() {

  /*  runs once during setup, fills array[10] with sensorvalues determined by index position x
      sets max and min values if sensorValue greater or lower
      calculates array average and variance after completion of array (x >= 9)
      also runs these calculations for each index position after the first complete array (y = 1)
      if variance less than 0.2, finish loop, else reset x to -1
      override by button press
      calculates calibration measures
  */

  // increase x from 0 to 9, x used as index value for array
  for (int x = -1 ; x < 10 ; x++) { // x = -1 as array starts at [0]

    // set maximum value
    if (sensorValue_air > sensorMaxCal) {
      sensorMaxCal = sensorValue_air;
    }

    // add readings to array
    array[x] = sensorValue_air;

    // if all indexes have a value (i.e. x = 9), calculate array average and variance
    // if y = 1, i.e. array indexed at least once, calculate array moving average and moving variance
    if (x >= 9 || y == 1) {
      avArray = ((array[0] + array[1] + array[2] + array[3] + array[4] + array[5] + array[6] + array[7] + array[8] + array[9]) / 10);
      varArray = ((pow((array[0] - avArray), expArray) + pow((array[1] - avArray), expArray) + pow((array[2] - avArray), expArray) +
                   pow((array[3] - avArray), expArray) + pow((array[4] - avArray), expArray) + pow((array[5] - avArray), expArray) +
                   pow((array[6] - avArray), expArray) + pow((array[7] - avArray), expArray) + pow((array[8] - avArray), expArray) +
                   pow((array[9] - avArray), expArray)) / 10);

      // if index filled, resest x for index loop
      if (x >= 9) {
        x = -1;
      }

      // if array variance less than 0.2, complete indexing else loop
      if (varArray < 0.2) {
        x = 10;
      } else {
        y = 1;
      }
    }

    // print moving variance valaue
    lcd.setCursor(0, 1);
    lcd.print("Variance = ");
    lcd.setCursor(11, 1);

    // if variance less than 100 and not equal to 0, print variance else print "na"
    if (varArray < 100 && varArray != 0) {
      lcd.print(varArray, 2);
    } else {
      lcd.print("na");
    }

    // if button pushed, override calibration and exit loop
    if (digitalRead(ButtonPin) == LOW) {
      x = 10;
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("OVERRIDE");
      delay(1000 * 2.5);
    }

    // set loop delay
    delay(1000);

  }

}

//-----warm up progress bar-----//
void WarmUp() {

  /*   reads potValue live, converts analog value to 0-60 scale
   *   potDelay converts 0-60 to 0-16 scale
   *   one loop = 1 block on LCD, additive process as no lcd.clear()
   *   loop fixed to 16 iterations so potDelay determines loop time
   *   minimum loop time of 62.5ms, i.e. 1 second
   */

  // block cursor defined by LCD width
  for (int blockCursor = 0; blockCursor < 17; blockCursor ++) {
    potValue = analogRead(pot);
    potMap = ((1023 - potValue) / 17.05); // value ranges 0-60
    lcd.setCursor(0, 0);
    lcd.print("Warming up...");

    // map 0-60 value to 0-16, multiply by 1000 to get milliseconds
    potDelay = (potMap / 16) * 1000;
    if (potDelay < 62.5) {
      potDelay = 62.5;
    }

    lcd.setCursor(blockCursor, 1);
    lcd.write(1023);
    delay(potDelay);

  }
}
