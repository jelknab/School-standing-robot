#include <MPU9250_WE.h>
#include <Wire.h>

#define MPU_INTERRUPT_PIN 3

#define MOTOR_LEFT_DIR 5
#define MOTOR_LEFT_STEP 6
#define MOTOR_LEFT_MS1 11
#define MOTOR_LEFT_MS2 10
#define MOTOR_LEFT_MS3 9

#define MOTOR_RIGHT_DIR 7
#define MOTOR_RIGHT_STEP 8
#define MOTOR_RIGHT_MS1 A1
#define MOTOR_RIGHT_MS2 A2
#define MOTOR_RIGHT_MS3 A3

#define POT_PIN A6

#define MOTOR_CONTROL_FREQUENCY 40000
#define SAMPLE_SIZE 3
#define PITCH_MEDIAN_INDEX SAMPLE_SIZE / 2

#define MAX_DEGREE_SECOND_CHANGE 45

MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

int samplingIndex = 0;
float pitchSamples[SAMPLE_SIZE];
float accelerationSamples[SAMPLE_SIZE];

bool pitchDataReady = false;

unsigned long motorLeftTiming = 18;
unsigned long motorRightTiming = 18;
unsigned long leftMotorTickCounter = 0;
unsigned long rightMotorTickCounter = 0;

unsigned short stepDeviderRanges[5] = {150, 120, 80, 40, 0};
int step_divider = 16;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  setup_IO();
  setup_MPU();
  setup_INTERRUPTS();
  
  Serial.println("Setup complete.");
}

void setup_IO() {
  pinMode(MOTOR_LEFT_STEP, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_MS1, OUTPUT);
  pinMode(MOTOR_LEFT_MS2, OUTPUT);
  pinMode(MOTOR_LEFT_MS3, OUTPUT);
  
  pinMode(MOTOR_RIGHT_STEP, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_MS1, OUTPUT);
  pinMode(MOTOR_RIGHT_MS2, OUTPUT);
  pinMode(MOTOR_RIGHT_MS3, OUTPUT);
}

void setup_MPU() {
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
    while(true) {}
  }


//  myMPU9250.setSampleRateDivider(1);
  
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  
  myMPU9250.setIntPinPolarity(MPU9250_ACT_HIGH); 
  myMPU9250.enableIntLatch(false);
  myMPU9250.enableInterrupt(MPU9250_DATA_READY);  

  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dataReadyISR, RISING);
}

void setup_INTERRUPTS() {
  // timer 1 (16-bit)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0; //initialize counter value to 0

  TCCR1B |= (1 << WGM12); // reset timer bij compare
  TCCR1B |= (1 << CS10); // prescaler van 1
  TIMSK1 |= (1 << OCIE1A); // zet timer compare interrupt aan
  
  OCR1A = 16000000 / MOTOR_CONTROL_FREQUENCY;
}

double errorPrior = 0;
double integral = 0;
unsigned long timestamp = 0;
void loop() {  
  float pot = analogRead(A6) / 1024.0;
  
  if(pitchDataReady) {
    pitchDataReady = false;
    
    double secondsPassed = ((micros() - timestamp) / 1000.0) / 1000.0;
    timestamp = micros();

    float measuredPitch = myMPU9250.getPitch() + 6;
    float measuredAcceleration = myMPU9250.getGyrValues().y;

//    pitchSamples[samplingIndex] = isnan(measuredPitch) ? 0 : measuredPitch;
//    accelerationSamples[samplingIndex] = isnan(measuredAcceleration) ? 0 : measuredAcceleration;
//    samplingIndex = (samplingIndex + 1) % SAMPLE_SIZE;

    double pitch = readPitch(measuredPitch, secondsPassed);
    double acceleration = readAcceleration(measuredAcceleration, secondsPassed);
      
    double currentPosition = acceleration*1.5 - pitch;

    double error = 0 - currentPosition;
    integral = min(integral + error * secondsPassed, 50);
    double derivative = (error - errorPrior) / secondsPassed;
    errorPrior = error;

    //             P           I               D
    float pidOut = 4*error + 1*integral + 0.02*derivative;

    Serial.print(pidOut);
    Serial.print(" ");
    Serial.println(currentPosition);
//    Serial.print(" ");
//    Serial.println((pot/10.0)*derivative);

    setMotorLeftRPM(pidOut);
    setMotorRightRPM(pidOut);
  }
}

double lastPitchReading = 0;
double readPitch(float pitch, double secondsPassed) {  
//  float samples_CPY[SAMPLE_SIZE];
  
//  memcpy(samples_CPY, pitchSamples, sizeof(float) * SAMPLE_SIZE);
//  qsort(samples_CPY, SAMPLE_SIZE, sizeof(float), compare_floats); 

//  float medianPitch = samples_CPY[PITCH_MEDIAN_INDEX];
  double difference = pitch - lastPitchReading;
  double changeAllowed = MAX_DEGREE_SECOND_CHANGE * secondsPassed;
  
  double newPitchLimited = lastPitchReading + max(-changeAllowed, min(difference, changeAllowed));
//  float newPitchSmoothed = lastPitchReading * 0.7 + newPitchLimited * 0.3;
  
  lastPitchReading = newPitchLimited;
  
  return newPitchLimited;
}

double lastAccReading = 0;
double readAcceleration(float accel, double secondsPassed) {
//  float samples_CPY[SAMPLE_SIZE];
  
//  memcpy(samples_CPY, accelerationSamples, sizeof(float) * SAMPLE_SIZE);
//  qsort(samples_CPY, SAMPLE_SIZE, sizeof(float), compare_floats); 

//  float medianAcc = samples_CPY[PITCH_MEDIAN_INDEX];
  double difference = accel - lastAccReading;
  double changeAllowed = MAX_DEGREE_SECOND_CHANGE * secondsPassed;

  double newAccLimited = lastAccReading + max(-changeAllowed, min(difference, changeAllowed));
//  float newAccSmoothed = lastAccReading * 0.7 + newAccLimited * 0.3;

  lastAccReading = newAccLimited;
  
  return newAccLimited;
}

unsigned long motorLeftDebounceTimestamp = 0;
bool leftDir = false;
void setMotorLeftRPM(float RPM) {
  float absRPM = min(abs(RPM), 150);
  
  //DIRECTION
  bool dir = RPM < 0;
  if (dir != leftDir) {
    if (millis() - motorLeftDebounceTimestamp > 0) {
      digitalWrite(MOTOR_LEFT_DIR, dir);
      motorLeftDebounceTimestamp = millis();

      leftDir = dir;
    } else {
      return;
    }
  }

  //STEP
  unsigned short stepDeviderRanges[5] = {125, 95, 80, 40, 0};
  for (int i = 0; i < 5; i++) {
    if (absRPM < stepDeviderRanges[i]) continue;
    
    step_divider = pow2(i);
    digitalWrite(MOTOR_LEFT_MS1, ((i & 1) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_LEFT_MS2, ((i & 2) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_LEFT_MS3, ((i & 4) > 0 || i == 4) ? HIGH : LOW);
    break;
  }

  //TIMING
  motorLeftTiming = (float) MOTOR_CONTROL_FREQUENCY / ((absRPM/60.0) * (200.0 * step_divider));
}

unsigned long motorRightDebounceTimestamp = 0;
bool rightDir = false;
void setMotorRightRPM(float RPM) {
  float absRPM = min(abs(RPM), 150);
  
  //DIRECTION
  bool dir = RPM > 0;
  if (dir != rightDir) {
    if (millis() - motorRightDebounceTimestamp > 0) {
      digitalWrite(MOTOR_RIGHT_DIR, dir);
      motorRightDebounceTimestamp = millis();

      rightDir = dir;
    } else {
      return;
    }
  }
  

  //STEP
  for (int i = 0; i < 5; i++) {
    if (absRPM < stepDeviderRanges[i]) continue;
    
    step_divider = pow2(i);
    digitalWrite(MOTOR_RIGHT_MS1, ((i & 1) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_RIGHT_MS2, ((i & 2) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_RIGHT_MS3, ((i & 4) > 0 || i == 4) ? HIGH : LOW);
    break;
  }

  //TIMING
  motorRightTiming = (float) MOTOR_CONTROL_FREQUENCY / ((absRPM/60.0) * (200.0 * step_divider));
}

int pow2(int p){
    return 1 << p;
}

int compare_floats(const void * a, const void * b)
{
  float fa = *(const float*) a;
  float fb = *(const float*) b;
  return (fa > fb) - (fa < fb);
}

void dataReadyISR() {
  pitchDataReady = true;
}

ISR(TIMER1_COMPA_vect) {  
  if (leftMotorTickCounter == 1) {
    digitalWrite(MOTOR_LEFT_STEP, LOW);    
  }
  if (++leftMotorTickCounter >= motorLeftTiming) { 
    leftMotorTickCounter = 0;
    digitalWrite(MOTOR_LEFT_STEP, HIGH);
  }

  if (rightMotorTickCounter == 1) {
    digitalWrite(MOTOR_RIGHT_STEP, LOW);    
  }
  if (++rightMotorTickCounter >= motorRightTiming) { 
    rightMotorTickCounter = 0;
    digitalWrite(MOTOR_RIGHT_STEP, HIGH);
  }
}
