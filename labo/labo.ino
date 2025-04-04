#include <HCSR04.h>
#include <LCD_I2C.h>
#include <AccelStepper.h>

#define TRIGGER_PIN 10
#define ECHO_PIN 12

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
LCD_I2C lcd(0x27, 16, 2);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

const char* DA = "2405238";
unsigned long currentTime = 0;
float distance = 0.0;
int degree = 10;
int minAngle = 10;
int maxAngle = 170;
long minStep = (minAngle * 2038.0) / 360;
long maxStep = (maxAngle * 2038.0) / 360;

const int DISPLAY_INTERVAL = 100;
const int DISTANCE_INTERVAL = 50;
const int SERIAL_INTERVAL = 100;
const int DOOR_MOVE_INTERVAL = 2000;

enum Etat {
  FERMEE,
  OUVERTURE,
  OUVERTE,
  FERMETURE
} currentState = FERMEE;

unsigned long doorMoveTime = 0;
bool isMoving = false;

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(500);
  //myStepper.setSpeed(250);
  myStepper.moveTo(minStep);
  myStepper.enableOutputs();

  departureDisplay();
}

void loop() {
  currentTime = millis();

  static unsigned long previousDistance = 0;
  if (currentTime - previousDistance >= DISTANCE_INTERVAL) {
    distance = hc.dist();
    previousDistance = currentTime;
  }
  
  stateManager();
  motorTask();
  screenDisplay(currentTime);
  serialPrint(currentTime); 6
}

void stateManager() {
  switch (currentState) {
    case FERMEE:
      if (distance < 30 && distance > 0) {
        currentState = OUVERTURE;
        doorMoveTime = currentTime;
        isMoving = true;
        myStepper.enableOutputs();
        myStepper.moveTo(maxStep);
      }
      break;

    case OUVERTURE:
      if (!isMoving) {
        currentState = OUVERTE;
        myStepper.disableOutputs();
      }
      break;

    case OUVERTE:
      if (distance > 60) {
        currentState = FERMETURE;
        doorMoveTime = currentTime;
        isMoving = true;
        myStepper.enableOutputs();
        myStepper.moveTo(minStep);
      }
      break;

    case FERMETURE:
      if (!isMoving) {
        currentState = FERMEE;
        myStepper.disableOutputs();
      }
      break;
  }
}

void motorTask() {
  if (isMoving) {
    myStepper.run();
    int currentPos = myStepper.currentPosition();
    degree = map(currentPos, minStep, maxStep, minAngle, maxAngle);
    if (!myStepper.isRunning()) {
      isMoving = false;
    }
  }
}

void departureDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(DA);
  lcd.setCursor(0, 1);
  lcd.print("Labo 4A");
  delay(2000);
  lcd.clear();
}

void screenDisplay(unsigned long ct) {
  static unsigned long previousDisplay = 0;
  if (ct - previousDisplay >= DISPLAY_INTERVAL) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print(" cm");

    lcd.setCursor(0, 1);
    switch (currentState) {
      case FERMEE:
        lcd.print("Porte: Fermee");
        break;
      case OUVERTE:
        lcd.print("Porte: Ouverte");
        break;
      case OUVERTURE:
      case FERMETURE:
        lcd.print("Porte: ");
        lcd.print(degree);
        lcd.print(" deg");
        break;
    }
    previousDisplay = ct;
  }
}

void serialPrint(unsigned long ct) {
  static unsigned long previousSerial = 0;
  if (ct - previousSerial >= SERIAL_INTERVAL) {
    Serial.print("etd:");
    Serial.print(DA);
    Serial.print(",dist:");
    Serial.print(distance);
    Serial.print(",deg:");
    Serial.println(degree);
    previousSerial = ct;
  }
}
