/*
 Name:		TeaTimer.ino
 Created:	20.11.2019 21:07:48
 Author:	cod3cruncher
*/

#include <Bounce2.h>
#include <EEPROM.h>
#include <Servo.h>
#include <TM1637Display.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16, 2); 

/*** ADDR EEPROM ***/
#define ADDR_LOWER 0
#define ADDR_UPPER 1

/*** STATES ***/
#define IDLE_STATE 1
#define CALIBRATE_LOWER_STATE 21
#define CALIBRATE_UPPER_STATE 22
#define BREWING_STATE 3

#define MIN_POT 0
#define MAX_POT 1023

#define MIN_BREWING_TIME 1
#define MAX_BREWING_TIME 10

#define MIN_SERVO_ANGLE 0
#define MAX_SERVO_ANGLE 180

#define CLK 2
#define DIO 3


/*** PINS ***/
#define IN_T 4
#define TAST_CALIBRATION 7
#define TAST_UPDOWN 12
#define TIME_POT_PIN 0
#define CALIBRATION_POT_PIN 1
#define SERVO_PIN 9
#define BRIGHTNESS_PIN 6

#define BUZZER_PIN 8

#define DISPLAY_BRIGHTNESS 1



Servo myservo;

TM1637Display display(CLK, DIO);

boolean isStartPressed;
boolean isFinished;
boolean isArmUp;
boolean isCalibratePressed = false;
boolean isUpDownPressed = false;

int angleLowerBound;
int angleUpperBound;

Bounce debouncerStart = Bounce();
Bounce debouncerCalibrate = Bounce();
Bounce debouncerUpDown = Bounce();

short state;

void setup() {
  lcd.begin();
  lcd.backlight();
  analogWrite(BRIGHTNESS_PIN, 50);
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  myservo.write(EEPROM.read(ADDR_UPPER));
  myservo.attach(SERVO_PIN);
  isArmUp = true;

  //  myservo.write(90);

  pinMode(BUZZER_PIN, OUTPUT) ;

  debouncerStart.attach(IN_T);
  debouncerStart.interval(5);
  debouncerCalibrate.attach(TAST_CALIBRATION);
  debouncerStart.interval(5);
  debouncerUpDown.attach(TAST_UPDOWN);
  debouncerUpDown.interval(5);


  state = IDLE_STATE;

  delay(100);
  //  armUp();
  Serial.begin(9600);
}


void loop() {


  // up down movement
  if (isCBtnUpDownClicked()) {
    if (isArmUp) {
      armDown();
    }
    else {
      armUp();
    }
  }


  //   the calibration mode
  if (isCalibrationBtnClicked()) {
    switch (state) {
      case CALIBRATE_LOWER_STATE:
        Serial.print("Lower Bound calibrated: ");
        Serial.println(angleLowerBound);
        state = CALIBRATE_UPPER_STATE;
        //        myservo.write(EEPROM.read(ADDR_UPPER));
        delay(100);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Obere Grenze");
        lcd.setCursor(0, 1);
        lcd.print("einstellen");
        break;
      case CALIBRATE_UPPER_STATE:
        Serial.print("Upper Bound calibrated: ");
        Serial.println(angleUpperBound);
        Serial.println("Calibration finished");
        Serial.print("LowerBound: ");
        Serial.print(angleLowerBound);
        Serial.print("   UpperBound: ");
        Serial.println(angleUpperBound);
        writeToEEPROM(ADDR_LOWER, angleLowerBound);
        writeToEEPROM(ADDR_UPPER, angleUpperBound);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Kalibrierung");
        lcd.setCursor(0, 1);
        lcd.print("beendet");
        delay(2000);
        lcd.clear();
        state = IDLE_STATE;
        break;
      case IDLE_STATE:
        state = CALIBRATE_LOWER_STATE;
        //        myservo.write(EEPROM.read(ADDR_LOWER));
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Untere Grenze");
        lcd.setCursor(0, 1);
        lcd.print("einstellen");
        Serial.println("Calibration started");
        delay(300);
    }
  }
  int servoAngle = getCalibrationServoAngle();
  switch (state) {
    case CALIBRATE_LOWER_STATE:
      angleLowerBound = servoAngle;
      myservo.write(servoAngle);
      //      Serial.print("Setting Lower Bound: ");
      //      Serial.println(servoAngle);
      break;
    case CALIBRATE_UPPER_STATE:
      angleUpperBound = servoAngle;
      myservo.write(servoAngle);
      //      Serial.print("Setting Upper Bound: ");
      //      Serial.println(servoAngle);
      break;
  }
  delay(10);
  int brewingTime = getBrewingTime();
  if (state == IDLE_STATE) {
    writeTimeToDisplay("Ziehzeit", brewingTime, 0);
    if (isStartBtnClicked()) {
      state = BREWING_STATE;
    }
    delay(20);
  }
  if (state == BREWING_STATE) {
    Serial.println("Timer started");
    armDown();
    for (int i = brewingTime; i > 0; i--) {
      writeTimeToDisplay("Restzeit", i, 0);
      for (int j = 59; j >= 0; j--) {
        delay(1000);
        Serial.print(i - 1);
        Serial.print(":");
        Serial.println(j);
        writeTimeToDisplay("Restzeit", i - 1, j);
      }
    }
    armUp();
    finished();
  }

}

/**
   checks if the start btn is pressed
*/
boolean isStartBtnClicked() {
  debouncerStart.update();
  int value = debouncerStart.read();
  if (isStartPressed && value == HIGH) {
    isStartPressed = false;
    Serial.println("Start Brewing clicked");
    return true;
  }
  if(!isStartPressed && value == LOW) {
    isStartPressed = true;
    return false;
  }
  return false;
}


/**
   checks if the calibration btn is pressed
*/
boolean isCalibrationBtnClicked() {
  debouncerCalibrate.update();
  int value = debouncerCalibrate.read();
  if (isCalibratePressed && value == HIGH) {
    isCalibratePressed = false;
    //    Serial.println("Calibration Button clicked");
    return true;
  }
  if (!isCalibratePressed && value == LOW) {
    isCalibratePressed = true;
    return false;
  }
  return false;
}


/**
   checks if the calibration btn is pressed
*/
boolean isCBtnUpDownClicked() {
  debouncerUpDown.update();
  int value = debouncerUpDown.read();
  if (isUpDownPressed && value == HIGH) {
    isUpDownPressed = false;
    Serial.println("UpDown Button clicked");
    return true;
  }
  if (!isUpDownPressed && value == LOW) {
    isUpDownPressed = true;
    return false;
  }
  return false;
}


/**
   returns the brewing time in minutes -
   the input value of the poti (0-1023) is already mapped correct
*/
int getBrewingTime() {
  return getMappedValue(analogRead(TIME_POT_PIN));
}


/**
   returns the angle for the servo
   the value is in range [0, 180]
*/
int getCalibrationServoAngle() {
  int potValue = analogRead(CALIBRATION_POT_PIN);
  return constrain(
           map(potValue, MIN_POT, MAX_POT, MAX_SERVO_ANGLE, MIN_SERVO_ANGLE),
           MIN_SERVO_ANGLE, MAX_SERVO_ANGLE
         );
}


/**
   mappes the pot values to the correct brewing range
*/
int getMappedValue(int potValue) {
  return constrain(
           map(potValue, MIN_POT, MAX_POT, MAX_BREWING_TIME, MIN_BREWING_TIME - 1),
           MIN_BREWING_TIME, MAX_BREWING_TIME
         );
}


void showTimeDisplay(int minutes) {
  showTimeDisplay(minutes, 0);
}


/**
   shows the minutes value on the display
*/
void showTimeDisplay(int minutes, int seconds) {
  //  display.showNumberDecEx(minutes * 100 + seconds, 0b11100000, false, 4, 0);
}


void finished() {
  Serial.println("Brewing finished!");
  state = IDLE_STATE;
  makeSound();
}


void armUp() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hebe Arm");
  int value =  EEPROM.read(ADDR_UPPER);
  for (int i = EEPROM.read(ADDR_LOWER); i <= value; i += 5) {
    myservo.write(i);
    delay(50);
  }
  //  myservo.write(value);
  Serial.print("Moving arm to ");
  Serial.println(value);
  isArmUp = true;
  delay(500);
  lcd.clear();
}


void armDown() {
  lcd.clear();
  lcd.print("Senke Arm");
  int value =  EEPROM.read(ADDR_LOWER);
  for (int i = EEPROM.read(ADDR_UPPER); i >= value; i -= 5) {
    myservo.write(i);
    delay(50);
  }
  //  myservo.write(value);
  Serial.print("Moving arm to ");
  Serial.println(value);
  isArmUp = false;
  delay(500);
  lcd.clear();
}

void writeTimeToDisplay(String label, int minutes, int seconds) {
  Serial.println(label);
  //  lcd.backlight();
  //  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(label);
  int cursorIndex = label.length();
  cursorIndex += 1;
  lcd.setCursor(cursorIndex, 0);
  if (minutes < 10) {
    lcd.print("0" + String(minutes));
  }
  else {
    lcd.print(minutes);
  }
  cursorIndex += 2;
  lcd.setCursor(cursorIndex, 0);
  lcd.print(":");
  cursorIndex++;
  lcd.setCursor(cursorIndex, 0);
  if (seconds < 10) {
    lcd.print("0" + String(seconds));
  }
  else {
    lcd.print(seconds);
  }
}


void makeSound() {
  tone(BUZZER_PIN, 261, 1000);
}


void writeToEEPROM(int addr, int value) {
  EEPROM.write(addr, value);
}
