// Stepper motor pins
const int stepPin = 2;
const int dirPin = 3;

// Microstepping mode pins
const int ms1Pin = 10;
const int ms2Pin = 9;
const int ms3Pin = 8;

// End switch pin
const int switchPin = 4;

// Potantiometer pins
const int potFrequencyPin = A0;
const int potAmplitudePin = A1;
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

  // Set MS1, MS2, MS3 pins
  digitalWrite(ms1Pin, HIGH);
  digitalWrite(ms2Pin, LOW);
  digitalWrite(ms3Pin, LOW);

  Serial.begin(9600);
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

void calibrate() {
  // Move to the edge until the needle activates the end switch
  while (digitalRead(switchPin) == HIGH) {
    digitalWrite(dirPin, LOW);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  // Move to the center
  digitalWrite(dirPin, HIGH);

  for (int i = 0; i < 4950; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(300);
  }

  delay(500);
}

void getPotValues() {
  int potAmplitudeValue = analogRead(potAmplitudePin);
  amplitude = map(potAmplitudeValue, 0, 1023, 20, 100);

  float potFrequencyValue = analogRead(potFrequencyPin);
  float maxFrequency = 20.0 / amplitude;
  float frequency = (float)potFrequencyValue / 1023 * (20.0 / amplitude);
  frequency = round(frequency / 0.02) * 0.02;

  int potWaveformValue = analogRead(potWaveformPin);
  selectedWaveform = map(potWaveformValue, 0, 1023, 0, 3);

  amplitude = 50;
  frequency = 0.1;
  selectedWaveform = 3;
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

  Serial.print("Steps to move: ");
  Serial.print(stepsToMove);

  // Determine the step delay
  int stepDelay = max(200, round(interval * 1000 / (2 * abs(stepsToMove))));

  Serial.print("   Step delay: ");
  Serial.println(stepDelay);

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