#include "HX711.h"

// Loadcell pins
#define DOUT 4
#define CLK 5

// Motor driver pins
#define AIN1 6
#define AIN2 8

// Encoder pins
#define ENCODER_A 2  // Interrupt 0
#define ENCODER_B 3  // Interrupt 1 for direction

HX711 scale;
float calibration_factor = 4180;  //For loadcell

// Motor speed (0 - 255)
const int threshold_emg = 75;
//const int motorSpeed = 220;
const int lsmotorSpeed = 220;
const int remotorSpeed = 180;
// Tension smoothing buffer
const int bufferSize = 3;
float tensionBuffer[bufferSize];
int bufferIndex = 0;
bool emg=false;
// Encoder tracking num
volatile long encoderCount = 0;

// Retraction delay logic
int ignoreIndex = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);  // Mandatory time for stabilized init

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

  //Setup Motor pins output
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
  //To stablize the closed-loop
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
  float avgTension = (-1) * sum / bufferSize;  // Invert if necessary(depends on the side of the load cell)

  // Safely read encoder count
  long count;
  noInterrupts();
  count = encoderCount;
  interrupts();

  // Data printing
  Serial.print("Avg Tension: ");
  Serial.print(avgTension, 2);
  Serial.print(" g | Encoder: ");
  Serial.println(count);

  if(emg) {
      analogWrite(AIN1, 255);
      digitalWrite(AIN2, LOW); 
      delay(150);
      emg = false;
  } // Decrement ignoreIndex each loop if active
  else{
  if (ignoreIndex > 0) {
    ignoreIndex--;
    Serial.println("Retraction delay active → motor paused");
  } else {
    // Hysteresis range(depends on the cable weight/tension when it's naturally hang)
    float lower_limit = 44.7;
    float upper_limit = 47.0;
    //check the direction of the rotation  
    if (avgTension > upper_limit) {
      //loosen
      if(avgTension > threshold_emg)
      {//Fast and strong drag
      analogWrite(AIN1, 255);
      digitalWrite(AIN2, LOW);
      //Serial.println("strongdrag");  
      delay(150);
      
      emg = true;
      }
      else{//normal drag scenerio
      
      analogWrite(AIN1, lsmotorSpeed);
      digitalWrite(AIN2, LOW);
      
      delay(60);
      Serial.println("normaldrag");
            

      }
    } else if (avgTension < lower_limit) {
      // tighten
      digitalWrite(AIN1, LOW);
      analogWrite(AIN2, remotorSpeed);
      //Serial.println("retract");
      delay(70);

      ignoreIndex = 0;  // Pause motor next loop-- preventing over-sensitive

      
    } else {
      // In range → Stop motor
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      //Serial.println('stop');
      delay(10);
    }
  }
  }
  delay(30);  // Loop timing--affects sensitivity 
}

void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a == b)
    encoderCount++;
  else
    encoderCount--;
}
void stopMotor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}