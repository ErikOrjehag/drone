
// Timer 0
int pwmPin1 = 5;
int pwmPin2 = 6;

// Timer 1 (dont use)
int pwmPin3 = 9;
int pwmPin4 = 10;

// Timer 2
int pwmPin0 = 3;
int pwmPin5 = 11;

int pwmLower = 70;
int pwmUpper = 125;

int inputPin = 0;

void setup() {
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

  Serial.begin(9600);
}

void loop() {
  int input = analogRead(inputPin);

  int pwm = map(input, 0, 1023, 70, 125);
  
  Serial.println(pwm);
  
  // put your main code here, to run repeatedly.
  analogWrite(pwmPin0, pwm);
  analogWrite(pwmPin1, pwm);
  analogWrite(pwmPin2, pwm);
  analogWrite(pwmPin3, pwm);
  analogWrite(pwmPin4, pwm);
  analogWrite(pwmPin5, pwm);
  /*delay(100);
  analogWrite(pwmPin0, pwmUpper);
  analogWrite(pwmPin1, pwmUpper);
  analogWrite(pwmPin2, pwmUpper);
  analogWrite(pwmPin3, pwmUpper);
  analogWrite(pwmPin4, pwmUpper);
  analogWrite(pwmPin5, pwmUpper);
  delay(100);*/
}
