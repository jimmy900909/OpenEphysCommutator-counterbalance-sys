#include "HX711.h"

// Load cell pins
#define DOUT 4
#define CLK 5

// Motor driver pins
#define AIN1 6
#define AIN2 8

// Encoder pins
#define ENCODER_A 2  // Interrupt 0
#define ENCODER_B 3  // Interrupt 1 (optional, for direction)

HX711 scale;
float calibration_factor = 4180;  // Adjust based on your load cell

// Motor speed (0 - 255)
const int motorSpeed = 220;
const int remotorSpeed = 200;
const int lsmotorSpeed = 140;
// Tension smoothing buffer
const int bufferSize = 4;
float tensionBuffer[bufferSize];
int bufferIndex = 0;

// Encoder tracking
volatile long encoderCount = 0;

// Retraction delay logic
int ignoreIndex = 0;

void setup() {
  Serial.begin(9600);
  delay(500);  // Let HX711 stabilize

  // Initialize HX711
  scale.begin(DOUT, CLK);
  Serial.println("Initializing HX711...");
  delay(2000);

  if (!scale.is_ready()) {
    Serial.println("HX711 not found.");
    while (1);
  }

  scale.set_scale(calibration_factor);
  scale.tare();
  Serial.println("HX711 ready.");

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // Encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // Initialize tension buffer
  for (int i = 0; i < bufferSize; i++) {
    tensionBuffer[i] = 0;
  }

  Serial.println("Setup complete.");
}

void loop() {
  if (!scale.is_ready()) {
    Serial.println("HX711 disconnected. Reinitializing...");
    scale.power_down();
    delay(100);
    scale.power_up();
    delay(500);
    scale.tare();  // Optional re-zero
  }

  // Read HX711 safely
  noInterrupts();
  float tension = scale.get_units(3);
  interrupts();

  // Update tension buffer
  tensionBuffer[bufferIndex] = tension;
  bufferIndex = (bufferIndex + 1) % bufferSize;

  // Calculate average tension
  float sum = 0;
  for (int i = 0; i < bufferSize; i++) {
    sum += tensionBuffer[i];
  }
  float avgTension = (-1) * sum / bufferSize;  // Invert if necessary

  // Safely read encoder count
  long count;
  noInterrupts();
  count = encoderCount;
  interrupts();

  // Print data
  Serial.print("Avg Tension: ");
  Serial.print(avgTension, 2);
  Serial.print(" g | Encoder: ");
  Serial.println(count);

  // Decrement ignoreIndex each loop if active
  if (ignoreIndex > 0) {
    ignoreIndex--;
    Serial.println("⏳ Retraction delay active → motor paused");
  } else {
    // Hysteresis bounds
    float lower_limit = 30.5;
    float upper_limit = 36.0;

    if (avgTension > upper_limit) {
      // Clockwise spin (tighten)
      analogWrite(AIN1, remotorSpeed);
      digitalWrite(AIN2, LOW);
    } else if (avgTension < lower_limit) {
      // Anticlockwise spin (loosen)
      digitalWrite(AIN1, LOW);
      analogWrite(AIN2, lsmotorSpeed);
      ignoreIndex = 1;  // Pause motor next loop
    } else {
      // In range → Stop motor
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
    }
  }

  delay(100);  // Loop timing
}

void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a == b)
    encoderCount++;
  else
    encoderCount--;
}
