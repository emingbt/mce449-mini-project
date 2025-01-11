#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Stepper motor pins
const int stepPin = 2;
const int dirPin = 3;

// Microstepping mode pins
const int ms1Pin = 4;

// Button Pins
const int resetCalibrateButtonPin = 6;
const int startStopButtonPin = 0;
const int waveformButtonPin = 5;
const int suprizeButtonPin = 1;

// Wave button pins
const int amplitudeIncreaseButtonPin = 10;
const int amplitudeDecreaseButtonPin = 11;
const int frequencyIncreaseButtonPin = 12;
const int frequencyDecreaseButtonPin = 13;

// End switch pin
const int switch1Pin = 8;
const int switch2Pin = 9;

// Time and motion variables
unsigned long previousTime = 0;     // Previous time (ms)
unsigned long interval = 100;       // Interval between steps (ms)

// Function parameters
int amplitude = 50;        // Maximum amplitude (mm)
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

  pinMode(switch1Pin, INPUT_PULLUP);
  pinMode(switch2Pin, INPUT_PULLUP);

  pinMode(resetCalibrateButtonPin, INPUT_PULLUP);
  pinMode(startStopButtonPin, INPUT_PULLUP);
  pinMode(waveformButtonPin, INPUT_PULLUP);

  pinMode(amplitudeIncreaseButtonPin, INPUT_PULLUP);
  pinMode(amplitudeDecreaseButtonPin, INPUT_PULLUP);
  pinMode(frequencyIncreaseButtonPin, INPUT_PULLUP);
  pinMode(frequencyDecreaseButtonPin, INPUT_PULLUP);

  // Start serial connection
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  initializeLCD(amplitude, frequency, selectedWaveform);

  // Set MS1 pin
  digitalWrite(ms1Pin, HIGH);

  // Read the total steps from EEPROM and calibrate the system according to it
  EEPROM.get(0, totalSteps);

  if (totalSteps <= 0) {
    initialCalibrate();
  } else {
    calibrate();
  }
}

void loop() {
  unsigned long currentTime = millis();  // Get the current time

  // Reset/Calibrate the system
  if (digitalRead(resetCalibrateButtonPin) == LOW) {
    pressStartTime = millis();  // Use millis() to get the current time

    while (digitalRead(resetCalibrateButtonPin) == LOW) {
      currentTime = millis();  // Update currentTime inside the loop

      if (currentTime - pressStartTime > calibrateInterval) {
        calibrate();
        pressStartTime = currentTime;  // Reset pressStartTime if needed
      }
    }

    if (previousPosition != 0) {
      reset(currentTime);
    }
  }

  // Start/Stop the system
  if (digitalRead(startStopButtonPin) == LOW) {
    isRunning = !isRunning;
    delay(300);
  }

  // Change the waveform
  if (digitalRead(waveformButtonPin) == LOW) {
    Serial.println("Waveform");
    selectedWaveform = (selectedWaveform != 3) ? (selectedWaveform + 1) : 0;
    String waveforms[4] = { "Sin", "Tri", "Sqr", "Saw" };

    lcd.setCursor(13, 1);
    lcd.print(waveforms[selectedWaveform]);

    reset(currentTime);
  }

  if (digitalRead(amplitudeIncreaseButtonPin) == LOW && amplitude < 70) {
    amplitude++;

    lcd.setCursor(0, 1);
    lcd.print(amplitude);

    delay(5);
  }

  if (digitalRead(amplitudeDecreaseButtonPin) == LOW && amplitude > 20) {
    amplitude--;

    lcd.setCursor(0, 1);
    lcd.print(amplitude);

    delay(5);
  }

  if (digitalRead(frequencyIncreaseButtonPin) == LOW && frequency < 1) {
    reset(currentTime);
    frequency += 0.01;

    lcd.setCursor(6, 1);
    lcd.print(frequency);

    delay(5);
  }

  if (digitalRead(frequencyDecreaseButtonPin) == LOW && frequency > 0.01) {
    reset(currentTime);
    frequency -= 0.01;

    lcd.setCursor(6, 1);
    lcd.print(frequency);

    delay(5);
  }

  // Time control for motion
  if (currentTime - previousTime >= interval && isRunning) {
    previousTime = currentTime;  // Update the time

    // Get position and move the motor
    float position = getPosition(amplitude, frequency, selectedWaveform, time);
    moveToPosition(position, previousPosition);

    // Advance time
    time += interval / 1000.0;
  }
}

void initialCalibrate() {
  // Move to the upper edge until the needle activates the limit switch
  digitalWrite(dirPin, LOW);

  while (digitalRead(switch2Pin) == HIGH) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  delay(500);

  // Move to the bottom edge and count the steps until the needle activates the limit switch
  digitalWrite(dirPin, HIGH);

  while (digitalRead(switch1Pin) == HIGH) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);

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

void calibrate() {  // With the known total steps, move to bottom edge first and move to the center
  digitalWrite(dirPin, HIGH);

  while (digitalRead(switch1Pin) == HIGH) {
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

  time = 0;
  previousPosition = 0;

  delay(1000);
}

void reset(float currentTime) {
  float mmPerStep = 0.04;
  int stepsToMove = round(previousPosition / mmPerStep);

  // Determine the direction
  bool isTurningClockwise = previousPosition < 0;
  digitalWrite(dirPin, isTurningClockwise ? HIGH : LOW);

  // Move to center
  if (stepsToMove != 0) {
    for (int i = 0; i < abs(stepsToMove); i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(1000);
    }
  }

  previousPosition = 0;
  time = 0;
  pressStartTime = currentTime;

  delay(500);
}

float getPosition(int amplitude, float frequency, int selectedWaveform, float time) {
  float position;

  switch (selectedWaveform) {
    case 0:
      position = sin(2 * PI * frequency * time);
      break;
    case 1:
      position = 2 * abs(fmod((time + 0.25) * frequency, 1.0) * 2 - 1) - 1;
      break;
    case 2:
      position = (sin(2 * PI * frequency * time) >= 0) ? 1.0 : -1.0;
      break;
    case 3:
      position = 2 * fmod((time + 0.5) * frequency, 1.0) - 1;
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

  int stepDelay = max(250, round(interval * 1000 / (2 * abs(stepsToMove))));

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

void initializeLCD(int amplitude, float frequency, int selectedWaveform) {
  String waveforms[4] = { "Sin", "Tri", "Sqr", "Saw" };

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