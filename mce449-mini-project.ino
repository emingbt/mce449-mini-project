#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Stepper motor pins
const int stepPin = 2;
const int dirPin = 3;

// Microstepping mode pins
const int ms1Pin = 10;
const int ms2Pin = 9;
const int ms3Pin = 8;

// End switch pin
const int switch1Pin = 4;
const int switch2Pin = 5;

// Potantiometer pins
const int potAmplitudePin = A0;
const int potFrequencyPin = A1;
const int potWaveformPin = A2;

// Button Pins
const int button1Pin = 5;
const int button2Pin = 6;

// Time and motion variables
unsigned long previousTime = 0;  // Previous time (ms)
unsigned long interval = 100;    // Interval between steps (ms)

// Sine function parameters
float amplitude = 50;      // Maximum amplitude (mm)
float frequency = 0.1;     // Frequency (Hz)
int selectedWaveform = 0;  // Waveform 0:Sine, 1:Triangle, 2:Square, 3:Sawtooth

float time = 0;

float previousPosition = 0;

int totalSteps = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Set pins as outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);
  pinMode(ms3Pin, OUTPUT);

  // Set pins as inputs
  pinMode(potFrequencyPin, INPUT);
  pinMode(potAmplitudePin, INPUT);
  pinMode(potWaveformPin, INPUT);

  pinMode(switch1Pin, INPUT_PULLUP);
  pinMode(switch2Pin, INPUT_PULLUP);
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  writeToLCD(amplitude, frequency, selectedWaveform);

  // Set MS1, MS2, MS3 pins
  digitalWrite(ms1Pin, HIGH);
  digitalWrite(ms2Pin, LOW);
  digitalWrite(ms3Pin, LOW);

  EEPROM.get(0, totalSteps);

  if (totalSteps <= 0) {
    initialCalibrate();
  } else {
    calibrate();
  }
}

void loop() {
  unsigned long currentTime = millis();  // Get the current time

  // Calibrate the system if calibrate button pressed
  if (digitalWrite(button2Pin) == LOW) {
    calibrate();
  }

  // Time control for motion
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;  // Update the time

    getPotValues();

    float position = getPosition(amplitude, frequency, selectedWaveform, time);

    moveToPosition(position, previousPosition);

    // Advance time
    time += interval / 1000.0;
  }
}

void initialCalibrate() {
  // Move to the upper edge until the needle activates the limit switch
  while (digitalRead(switch1Pin) == HIGH) {
    digitalWrite(dirPin, LOW);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1500);
  }

  delay(500);

  // Move to the bottom edge and count the steps until the needle activates the limit switch
  while (digitalRead(switch2Pin) == HIGH) {
    digitalWrite(dirPin, HIGH);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1500);

    totalSteps++;
  }

  delay(1000);

  //Store the total counted steps to the eeprom and move to the center
  EEPROM.put(0, totalSteps);

  digitalWrite(dirPin, LOW);

  for (int i = 0; i < (totalSteps / 2); i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(600);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(600);
  }

  delay(1000);
}

void calibrate() { // With the known total steps, move to bottom edge first and move to the center
  while (digitalRead(switch2Pin) == HIGH) {
    digitalWrite(dirPin, HIGH);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1500);
  }

  digitalWrite(dirPin, LOW);

  for (int i = 0; i < (totalSteps / 2); i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(600);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(600);
  }

  delay(1000);
}

void getPotValues() {
  int potAmplitudeValue = analogRead(potAmplitudePin);
  amplitude = map(potAmplitudeValue, 0, 1023, 20, 100);

  float potFrequencyValue = analogRead(potFrequencyPin);
  float maxFrequency = 20.0 / amplitude;
  frequency = (float)potFrequencyValue / 1023 * (20.0 / amplitude);
  frequency = round(frequency / 0.02) * 0.02;

  int potWaveformValue = analogRead(potWaveformPin);
  selectedWaveform = map(potWaveformValue, 0, 1023, 0, 3);

}

float getPosition(int amplitude, float frequency, int selectedWaveform, float time) {
  float position;

  switch (selectedWaveform) {
    case 0:
      position = sin(2 * PI * frequency * time);
      break;
    case 1:
      position = 2 * abs(fmod(time * frequency, 1.0) * 2 - 1) - 1;
      break;
    case 2:
      position = (sin(2 * PI * frequency * time) >= 0) ? 1.0 : -1.0;
      break;
    case 3:
      position = 2 * fmod(time * frequency, 1.0) - 1;
      break;
    default:
      position = 0;
  }

  return position * amplitude;
}

void moveToPosition(float position, float currentPosition) {
  float mmPerStep = 0.04;
  float nextPosition = position;
  float mmToMove = nextPosition - currentPosition;
  int stepsToMove = round(mmToMove / mmPerStep);

  // Determine the direction
  bool isTurningClockwise = nextPosition > currentPosition;
  digitalWrite(dirPin, isTurningClockwise ? HIGH : LOW);


  // Determine the step delay
  int stepDelay = max(300, round(interval * 1000 / (2 * abs(stepsToMove))));


  // Move only if steps are needed
  if (stepsToMove != 0) {
    for (int i = 0; i < abs(stepsToMove); i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
    }
  }

  // Update the current position
  previousPosition = nextPosition;
}

void writeToLCD(int amplitude, float frequency, int selectedWaveform) {
  String waveforms[4] = {"Sin", "Tri", "Sqr", "Saw"};

  lcd.setCursor(0, 0);
  lcd.print("Amp");
  lcd.setCursor(6, 0);
  lcd.print("Freq");
  lcd.setCursor(13, 0);
  lcd.print("W.F");
  lcd.setCursor(0, 1);
  lcd.print(amplitude);
  lcd.setCursor(6, 1);
  lcd.print(frequency);
  lcd.setCursor(13, 1);
  lcd.print(waveforms[selectedWaveform]);
}