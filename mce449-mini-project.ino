// Stepper motor pins
const int stepPin = 2;
const int dirPin = 3;

// Microstepping mode pins
const int ms1Pin = 10;
const int ms2Pin = 9;
const int ms3Pin = 8;

// Potantiometer pins
const int potFrequencyPin = A0;
const int potAmplitudePin = A1;
const int potWaveformPin = A2;

// Time and motion variables
unsigned long previousTime = 0;  // Previous time (ms)
unsigned long interval = 100;    // Interval between steps (ms)

// Sine function parameters
float amplitude = 50;   // Maximum amplitude (mm)
float frequency = 0.1;  // Frequency (Hz)

float time = 0;

float previousPosition = 0;

void setup() {
  // // Set pins as outputs
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

  // Time control for motion
  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;  // Update the time

    // // For future potantiometer connections;
    int potAmplitudeValue = analogRead(potAmplitudePin);
    int updatedAmplitude = map(potAmplitudeValue, 0, 1023, 20, 100);

    float potFrequencyValue = analogRead(potFrequencyPin);
    float maxFrequency = 20.0 / amplitude;
    float updatedFrequency = (float)potFrequencyValue / 1023 * (20.0 / updatedAmplitude);
    updatedFrequency = round(updatedFrequency / 0.02) * 0.02;

    int potWaveformValue = analogRead(potWaveformPin);
    int selectedWaveform = map(potWaveformValue, 0, 1023, 0, 3);

    // // Debugging information
    // Serial.print("Curr time: ");
    // Serial.println(currentTime);
    // Serial.print("Amplitude: ");
    // Serial.println(updatedAmplitude);
    // Serial.print("Frequency: ");
    // Serial.println(updatedFrequency);

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

    moveToPosition(position, amplitude, previousPosition);

    // Advance time
    time += interval / 1000.0;
  }
}


void moveToPosition(float position, float amplitude, float currentPosition) {
  float mmPerStep = 0.04;
  float nextPosition = position * amplitude;
  float mmToMove = nextPosition - currentPosition;
  int stepsToMove = round(mmToMove / mmPerStep);

  // Determine the direction
  bool isTurningClockwise = nextPosition > currentPosition;
  digitalWrite(dirPin, isTurningClockwise ? HIGH : LOW);

  int stepDelay = max(150, round(interval * 1000 / (2 * abs(stepsToMove))) - 1);

  // // Debugging information
  // Serial.print("Next Position: ");
  // Serial.println(nextPosition);
  // Serial.print("Current Position: ");
  // Serial.println(currentPosition);
  // Serial.print("MM to Move: ");
  // Serial.println(abs(mmToMove));
  // Serial.print("MM per Step: ");
  // Serial.println(abs(mmPerStep));
  // Serial.print("Steps to Move: ");
  // Serial.println(abs(stepsToMove));
  // Serial.print("Direction: ");
  // Serial.println(isTurningClockwise ? "Clockwise" : "Counterclockwise");
  // Serial.print("Step delay: ");
  // Serial.println(stepDelay);

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
