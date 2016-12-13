#include <PID_v1.h>

#include "Gyro.h"

int MANUAL_MODE = 0;
int YOLO_MODE = 1;
int MODE = YOLO_MODE;

Gyro gyro;

double setpoint = 0, input = 0, output;
float error = 0, delta_error = 0;

// Specify the links and initial tuning parameters
PID pid(&input, &output, &setpoint, 1, 0, 0, DIRECT);

// Timer 0
int pwmPin1 = 5;
int pwmPin2 = 6;

// Timer 1 (dont use)
int pwmPin3 = 9;
int pwmPin4 = 10;

// Timer 2
int pwmPin0 = 3;
int pwmPin5 = 11;

int MOTOR[4];

int pwmLower = 77;
int pwmUpper = 125;

int inputPin = 0;

long start_ts;

void calibrate_all_motors(int MOTOR[4], int pwmLower, int pwmUpper) {
  for(int k = 0; k < 4; ++k) {  
    analogWrite(MOTOR[k], pwmUpper);
  }
  Serial.println("BATTERY!");
  delay(5000);
  Serial.println("GO!");
  for(int k = 0; k < 4; ++k) {
    analogWrite(MOTOR[k], pwmLower);
  }
}

void start_motors(int MOTOR[4]) {
  for(int k = 0; k < 4; ++k){
    analogWrite(MOTOR[k], pwmLower);
  }
  delay(100);
}

void setup() {
  Serial.begin(9600);

  MOTOR[0] = 11;
  MOTOR[1] = 6;
  MOTOR[2] = 5;
  MOTOR[3] = 3;
  
  // Change PWM frequency to something that the ESC understands.
  TCCR0B = (TCCR0B & 0b11111000) | 0x04;
  TCCR1B = (TCCR1B & 0b11111000) | 0x04;
  TCCR2B = (TCCR2B & 0b11111000) | 0x05;
  
  pinMode(inputPin, INPUT);
  
  pinMode(pwmPin0, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
  pinMode(pwmPin5, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(50);

  /*
  Serial.println("calibrate motors");
  Serial.println(3);
  delay(1000);
  Serial.println(2);
  delay(1000);
  Serial.println(1);
  delay(1000);
  Serial.println(0);
  calibrate_motors(MOTOR, pwmLower, pwmLower);
  Serial.println("calibrated motors done");

  delay(1000000);
  */
  
  if( MODE == YOLO_MODE ){

    Serial.println("Start motors");
    start_motors(MOTOR);
    Serial.println("Motors started!");
  
    Serial.println("calibrate");
    gyro.init();
    Serial.println("calibrated done");
  }

  start_ts = millis();

}

float calc_pid(float error, float delta_error, float P, float D) {
  return P * error + D * delta_error;
}

#define NUM_VALUES 20
float angles[2][NUM_VALUES];
int angle_count = 0;

float mean(float array[2][NUM_VALUES], int num) {
  float m = 0;
  for(int k = 0; k < NUM_VALUES; ++k)
    m += array[num][k];
  return m / NUM_VALUES;
}

void loop() {
  if( MODE == MANUAL_MODE ) {
    int input = analogRead(inputPin);
    int pwm = map(input, 0, 1023, pwmLower, pwmUpper + 5);
    for(int k = 0; k < 4; ++k){
      analogWrite(MOTOR[k], pwm);
    }
    Serial.println(pwm);
  }
  else if (MODE == YOLO_MODE) {
  
    //Serial.println(pwm);
    gyro.update();
  
    float angle_x = max(-20, min(20, asin(gyro.acc_x / 9.82) * 180 / PI));
    float angle_y = max(-20, min(20, asin(gyro.acc_y / 9.82) * 180 / PI));

    angles[0][angle_count] = angle_x;
    angles[1][angle_count] = angle_y;
    angle_count = (angle_count + 1) % NUM_VALUES;

    angle_x = mean(angles,0);
    angle_y = mean(angles,1);
  
    /*float output_x = calc_pid(-angle_x, 0, 1, 0);
    float output_y = calc_pid(-angle_y, 0, 1, 0);*/
  
    /*float corr_x = map(angle_x, 0, 45, pwmLower, pwmLower + (pwmUpper - pwmLower)/2);
    float corr_y = map(angle_y, 0, 45, pwmLower, pwmLower + (pwmUpper - pwmLower)/2);*/
  
    float corr_x = map(angle_x, 0, 20, 0, -40);
    float corr_y = map(angle_y, 0, 20, 0, -40);

    int base = min(91, pwmLower + (millis() - start_ts) / 100.0);

    Serial.println(base);
    
    /*
    analogWrite(MOTOR[0], base + corr_x);
    analogWrite(MOTOR[1], base + corr_x);
    analogWrite(MOTOR[2], base - corr_x);
    analogWrite(MOTOR[3], base - corr_x);
    */
    /*
    analogWrite(MOTOR[0], base + corr_y);
    analogWrite(MOTOR[1], base - corr_y);
    analogWrite(MOTOR[2], base - corr_y);
    analogWrite(MOTOR[3], base + corr_y);
    */
    
    analogWrite(MOTOR[0], base + corr_x + corr_y);
    analogWrite(MOTOR[1], base + corr_x - corr_y);
    analogWrite(MOTOR[2], base - corr_x - corr_y);
    analogWrite(MOTOR[3], base - corr_x + corr_y);
    
    //Serial.print("angle_x = "); Serial.print(angle_x); Serial.print(", angle_y = "); Serial.print(angle_y); Serial.println("");
    //analogWrite(MOTOR_1, base + corr_x - corr_y);
    //analogWrite(MOTOR_2, base - corr_x + corr_y);
    //nalogWrite(MOTOR_3, base - corr_x - corr_y);
  }
  
  //int pwm = map(angle, 0, 45, pwmLower, pwmLower + (pwmUpper - pwmLower) / 2);
  /*
  Serial.print(angle);
  Serial.print(", ");
  Serial.println(output);
  */
  /*
  input = angle;
  pid.Compute();
  double output1 = output;
  input = -angle;
  pid.Compute();
  double output2 = -output;
  Serial.print(angle);
  Serial.print(", ");
  if( angle <= 0 )
    Serial.println(output1);
  else
    Serial.println(output2);
    */
  
  
  // put your main code here, to run repeatedly.
  /*
  analogWrite(pwmPin0, pwmLower);
  analogWrite(pwmPin1, pwmLower); //
  analogWrite(pwmPin2, pwmLower);
  analogWrite(pwmPin3, pwmLower);
  analogWrite(pwmPin4, pwmLower);
  analogWrite(pwmPin5, pwmLower); //
  */
  /*delay(100);
  analogWrite(pwmPin0, pwmUpper);
  analogWrite(pwmPin1, pwmUpper);
  analogWrite(pwmPin2, pwmUpper);
  analogWrite(pwmPin3, pwmUpper);
  analogWrite(pwmPin4, pwmUpper);
  analogWrite(pwmPin5, pwmUpper);
  delay(100);*/
}
