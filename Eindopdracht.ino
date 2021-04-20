#include <MPU9250_WE.h>
#include <CircularBuffer.h>
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

#define MOTOR_CONTROL_FREQUENCY 50000
#define SAMPLE_SIZE 3
#define PITCH_MEDIAN_INDEX SAMPLE_SIZE / 2

#define KP 1
#define KI 0
#define KD 0

MPU9250_WE myMPU9250 = MPU9250_WE(0x68);
CircularBuffer<float, SAMPLE_SIZE> pitchSampleBuffer;

int samplingIndex = 0;
float pitchSamples[SAMPLE_SIZE];
float accelerationSamples[SAMPLE_SIZE];

bool pitchDataReady = false;

unsigned long motorLeftTiming = 18;
unsigned long motorRightTiming = 18;
unsigned long leftMotorTickCounter = 0;
unsigned long rightMotorTickCounter = 0;

unsigned short stepDeviderRanges[5] = {125, 95, 80, 40, 0};
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


  myMPU9250.setSampleRateDivider(2);
  
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_5);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_5);
  
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

bool enabled = false;
unsigned long timestamp = 0;
void loop() {  
  if(pitchDataReady) {
    pitchDataReady = false;

    byte index = (samplingIndex++) % SAMPLE_SIZE;
    if (index == 0) {enabled = true;}

    float sampledPitch = myMPU9250.getPitch();
    float sampledAcceleration = myMPU9250.getGyrValues().y;
    pitchSamples[samplingIndex] = sampledPitch;
    accelerationSamples[samplingIndex] = sampledAcceleration;
    samplingIndex = (samplingIndex + 1) % SAMPLE_SIZE;
    
    if (enabled) {
      float pitch = readPitch();
      float acceleration = readAcceleration();

      float pitchABS = abs(pitch);
      float uprightAdjustment = ((pitchABS * pow(pitchABS, -0.5)) / (90 * pow(90, -0.5))) * 60;
      uprightAdjustment = (pitch < 0) ? -uprightAdjustment : uprightAdjustment;
      
      float accelerationAdjustment = max(-30, min(-acceleration, 30));
      float targetRPM = historyRPM * 0.8 + (accelerationAdjustment + uprightAdjustment) * 0.2;
      if (isnan(targetRPM)){
        return;
      }
      Serial.println(targetRPM);

      historyRPM = targetRPM;

      setMotorLeftRPM(targetRPM);
      setMotorRightRPM(targetRPM);

    }
  }
}

float readPitch() {  
  float samples_CPY[SAMPLE_SIZE];
  
  memcpy(samples_CPY, pitchSamples, sizeof(float) * SAMPLE_SIZE);
  qsort(samples_CPY, SAMPLE_SIZE, sizeof(float), compare_floats); 
  
  return samples_CPY[PITCH_MEDIAN_INDEX];
}

float readAcceleration() {
  float samples_CPY[SAMPLE_SIZE];
  
  memcpy(samples_CPY, accelerationSamples, sizeof(float) * SAMPLE_SIZE);
  qsort(samples_CPY, SAMPLE_SIZE, sizeof(float), compare_floats); 
  
  return samples_CPY[PITCH_MEDIAN_INDEX];
}

unsigned long motorLeftDebounceTimestamp = 0;
bool leftDir = false;
void setMotorLeftRPM(float RPM) {
  float absRPM = abs(RPM);
  
  //DIRECTION
  bool dir = RPM < 0;
  if (dir != leftDir) {
    if (millis() - motorLeftDebounceTimestamp > 25) {
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
  float absRPM = abs(RPM);
  
  //DIRECTION
  bool dir = RPM > 0;
  if (dir != rightDir) {
    if (millis() - motorRightDebounceTimestamp > 25) {
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
