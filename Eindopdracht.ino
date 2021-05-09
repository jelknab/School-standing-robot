#include <MPU9250_WE.h>
#include <IRremote.h>
#include <Wire.h>

#define MPU_INTERRUPT_PIN 3
#define IR_RECEIVE_PIN 11

#define MOTOR_LEFT_DIR 5
#define MOTOR_LEFT_STEP 6
#define MOTOR_LEFT_MS1 12
#define MOTOR_LEFT_MS2 10
#define MOTOR_LEFT_MS3 9

#define MOTOR_RIGHT_DIR 7
#define MOTOR_RIGHT_STEP 8
#define MOTOR_RIGHT_MS1 A1
#define MOTOR_RIGHT_MS2 A2
#define MOTOR_RIGHT_MS3 A3

#define POT_PIN A6

#define MOTOR_CONTROL_FREQUENCY 50000
#define MAX_DEGREE_SECOND_CHANGE 10

// 1 rotation = (2*8.5 + 2*3.5) * PI = ~75.40 cm
// 1 wheel rotation = 18.85 cm
// 90 deg rotation = 75.40 / 4 = 18.85 wheel rotation

MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

bool mpuDataReady = false;

unsigned long motorLeftTiming = 0;
unsigned long motorRightTiming = 0;
unsigned long leftMotorTickCounter = 0;
unsigned long rightMotorTickCounter = 0;

float motorLeftRPM = 0;
float motorRightRPM = 0;

float leftTravelled = 0;
float leftTargetTravel = 0;
float rightTravelled = 0;
float rightTargetTravel = 0;

unsigned short stepDeviderRanges[5];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  

  Serial.println("LEFT_TRAVEL LEFT_PID RIGHT_TRAVEL RIGHT_PID");

  setup_IR();
  setup_STEPS();
  setup_IO();
  setup_MPU();
  setup_INTERRUPTS();
}

void setup_IR() {
  IrReceiver.begin(IR_RECEIVE_PIN);
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

// Calculates at which speed which step size should be used
void setup_STEPS() {
  for (int i = 0; i < 5; i++) {
    int step_divide = 1 << i;

    for (int rpm = 1; rpm < 500; rpm++) {
      int currTicks = calculateTicks(rpm, step_divide);
      int nextTicks = calculateTicks(rpm + 1, step_divide);

      if (currTicks == nextTicks) {
        stepDeviderRanges[i] = rpm;
        break;
      }
    }
  }

  stepDeviderRanges[4] = 0;
}

void setup_MPU() {
  while(!myMPU9250.init()) {}

  myMPU9250.setGyrOffsets(0.0f, 1000.0f, 0.0f);
  
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


unsigned long timestamp = 0;
void loop() {      
  if(mpuDataReady) {
    mpuDataReady = false;
    
    double secondsPassed = ((micros() - timestamp) / 1000.0) / 1000.0;
    timestamp = micros();

//    leftTravelled += motorLeftRPM * secondsPassed;    
//    rightTravelled += motorRightRPM * secondsPassed;


//    double leftPID = calculateLeftMotorPID(secondsPassed);
//    double rightPID = calculateRightMotorPID(secondsPassed);

//    Serial.print(leftTravelled);
//    Serial.print(" ");
//    Serial.println(leftPID);

//    double lean = -(leftPID + rightPID) / 10;
    double uprightPID = calculateUprightPID(secondsPassed, 0);
    
    setMotorLeftRPM(uprightPID);
    setMotorRightRPM(uprightPID);
  }

  if (IrReceiver.decode()) {
    
    switch(IrReceiver.decodedIRData.command) {
      case 88: // forward
        reset();
        leftTargetTravel = -37.7;
        rightTargetTravel = -37.7;
      break;
      case 89: // backward
        reset();
        leftTargetTravel = 37.7;
        rightTargetTravel = 37.7;
      break;
      case 90: // left
        reset();
        leftTargetTravel = -37.7;
        rightTargetTravel = 37.7;
      break;
      case 91: //right
        reset();
        leftTargetTravel = 37.7;
        rightTargetTravel = -37.7;
      break;
    }


    IrReceiver.resume();
  }
}

void reset() {
    leftTravelled = 0;
    leftTargetTravel = 0;
    rightTravelled = 0;
    rightTargetTravel = 0;
}

double leftErrorPrior = 0;
double leftIntegral = 0;
double calculateLeftMotorPID(double secondsPassed) {
  double error = constrain(leftTargetTravel - leftTravelled, -10, 10);
  leftIntegral = leftIntegral + error * secondsPassed;
  double derivative = (error - leftErrorPrior) / secondsPassed;
  leftErrorPrior = error;

  return 1*error + .5*leftIntegral + 0.15*derivative;
}

double rightErrorPrior = 0;
double rightIntegral = 0;
double calculateRightMotorPID(double secondsPassed) {
  double error = constrain(rightTargetTravel - rightTravelled, -10, 10);
  rightIntegral = rightIntegral + error * secondsPassed;
  double derivative = (error - rightErrorPrior) / secondsPassed;
  rightErrorPrior = error;

  return 1*error + .5*rightIntegral + 0.15*derivative;
}

double uprightErrorPrior = 0;
double uprightIntegral = 0;
double calculateUprightPID(double secondsPassed, double lean) {
    float measuredPitch = myMPU9250.getPitch();
    float measuredAcceleration = myMPU9250.getGyrValues().y;

    double pitch = filterPitch(measuredPitch, secondsPassed);
    double acceleration = filterAcceleration(measuredAcceleration, secondsPassed);
      
    double currentPosition = acceleration * 0.5 - pitch * 0.5;

    double error = lean - currentPosition;
    uprightIntegral = constrain(uprightIntegral + error * secondsPassed, -1, 1);
    double derivative = (error - uprightErrorPrior) / secondsPassed;
    uprightErrorPrior = error;

    //       P               I                    D
    return 5*error + 95*uprightIntegral + 0.05*derivative;
}

double pitchPrior = 0;
double filterPitch(float rawPitch, double secondsPassed) {  
  double difference = rawPitch - pitchPrior;
  double changeAllowed = min(MAX_DEGREE_SECOND_CHANGE * secondsPassed, 0.1);
  
  pitchPrior += difference * changeAllowed;
  
  return pitchPrior;
}

double accelerationPrior = 0;
double filterAcceleration(float rawAcceleration, double secondsPassed) {
  double difference = rawAcceleration - accelerationPrior;
  double changeAllowed = min(MAX_DEGREE_SECOND_CHANGE * secondsPassed, 0.1);

  accelerationPrior += difference * changeAllowed;
  
  return accelerationPrior;
}

void setMotorLeftRPM(float RPM) {  
  motorLeftRPM = RPM;
  float absRPM = min(abs(RPM), 100);

  digitalWrite(MOTOR_LEFT_DIR, RPM < 0);

  //STEP
  int step_divider = 16;
  for (int i = 0; i < 5; i++) {
    if (absRPM < stepDeviderRanges[i]) continue;
    
    step_divider = pow2(i);
    digitalWrite(MOTOR_LEFT_MS1, ((i & 1) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_LEFT_MS2, ((i & 2) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_LEFT_MS3, ((i & 4) > 0 || i == 4) ? HIGH : LOW);
    break;
  }

  //TIMING
  motorLeftTiming = calculateTicks(absRPM, step_divider);
}

void setMotorRightRPM(float RPM) {
  motorRightRPM = RPM;
  float absRPM = min(abs(RPM), 100);
  
  //DIRECTION
  digitalWrite(MOTOR_RIGHT_DIR, RPM > 0);  

  //STEP
  int step_divider = 16;
  for (int i = 0; i < 5; i++) {
    if (absRPM < stepDeviderRanges[i]) continue;
    
    step_divider = pow2(i);
    digitalWrite(MOTOR_RIGHT_MS1, ((i & 1) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_RIGHT_MS2, ((i & 2) > 0 || i == 4) ? HIGH : LOW);
    digitalWrite(MOTOR_RIGHT_MS3, ((i & 4) > 0 || i == 4) ? HIGH : LOW);
    break;
  }

  //TIMING
  motorRightTiming = calculateTicks(absRPM, step_divider);
}

int calculateTicks(float absRPM, int step_divider) {
  return (double) MOTOR_CONTROL_FREQUENCY / ((absRPM/60.0) * (200.0 * step_divider));
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
  mpuDataReady = true;
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
