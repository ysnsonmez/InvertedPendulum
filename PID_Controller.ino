/************************************
    Inverted Pendulum Controller
*                                   *
           Design Poject
*                                   *
        Yasin SÖNMEZ 2015
*                                   *
************************************/


#include <PID_v1.h>

/* ARDUİNO PİNS */

/* voltage motors with PWM pins %0 - %100 */
const int directionPwmPin1 = 5;
const int directionPwmPin2 = 6;

/* motor enable pins */
const int enablePin = 4;

/* read the potantiometer (encoder) pin */
const int encoderPin = A0;

/* CONSTANT VALUES */

/* desired angle value */
const int desiredAngle = 0;

/* VARİABLES */

/* motor direction of movement */
int motorDirection = 1;

/* potantiometer(encoder) sends raw values 0-1023 */
int encoderPosition = 0;

/* degrees 30 - 315 */
float encoderAngle = 0;

/* -179 <= relativeAngle <= 180: pendulum angle difference from top of arc */
float relativeAngle = 0;

/* difference between real angle and desired angle */
float error = 0;

/* PID VARIABLES */
double pidSetpoint, pidInput, pidOutput;
double realOutput;

/* create the PID controller */
//PID myPID(&pidInput, &pidOutput, &pidSetpoint, 2, 5, 1, DIRECT); // standard tuning
PID myPID(&pidInput, &pidOutput, &pidSetpoint, 3.6, 1.6, 0.65, DIRECT); // debug tuning

/*
  // frontend
  double Setpoint = desiredAngle;
  double Input = encoderPin;
  double Output = enablePin;
  unsigned long serialTime;
*/

/* reverse the motor direction */
void changeDirection(int pwm)
{
  if (motorDirection == 1)
  {
    analogWrite(directionPwmPin1, pwm);
    analogWrite(directionPwmPin2, 0);
  }
  else
  {
    analogWrite(directionPwmPin1, 0);
    analogWrite(directionPwmPin2, pwm);
  }
}

void setup()
{
  /* output pins set */
  pinMode(directionPwmPin1, OUTPUT);
  pinMode(directionPwmPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  /* start with 0 motor power */
  digitalWrite(enablePin, LOW);

  /* set the PID variables */
  pidInput = 0;
  pidSetpoint = desiredAngle;

  /* turn PID on */
  myPID.SetMode(AUTOMATIC);

  /* This range (-128,128) rotary motor with 50% power. */
  myPID.SetOutputLimits(-128.0, 128.0);

  /* set the serial communication speed */
  Serial.begin(9600);
}

void loop() {
  delay(1);

  /* read potantiometer(encoder) and compute angle */
  encoderPosition = analogRead(encoderPin);

  /* translate 0-1023 from encoder 30-315 degrees */
  encoderPosition = map(encoderPosition, 0, 1024, 30, 315);

  /* pendulum angle = encoderangle */
  encoderAngle = encoderPosition - 180;

  relativeAngle = encoderAngle;


  /* difference relative to the desired angle.*/
  error = relativeAngle;

  /* set the PID input */
  pidInput = error;
  myPID.Compute();


  /* minimum motor power is 50% */
  if (pidOutput == 0)
  {
    realOutput = 0;
  }
  else if (pidOutput > 0 && error < 0)
  {
    realOutput = pidOutput + 127;
    motorDirection = 1;
    changeDirection(abs(realOutput));
  }
  else if (pidOutput < 0 && error > 0 )
  {
    realOutput = pidOutput - 127;
    motorDirection = 0;
    changeDirection(abs(realOutput));
  }
  
  //delay(500);

  encoderPosition = analogRead(encoderPin);
  encoderPosition = map(encoderPosition, 0, 1024, 30, 315);
  encoderAngle = encoderPosition - 180;
  relativeAngle = encoderAngle;


  // turn of the PID and motor if the pendulum vertical. -/+5 degree error tolerance.
  if (relativeAngle < 3 && relativeAngle > -3)
    //if(error == 0)
  {
    Serial.print("/ SARKAC DIKEY KONUMDA / ");
    myPID.SetMode(MANUAL);
    digitalWrite(enablePin, LOW);
    pidOutput = 0;
    realOutput = 0;
  }
  else
  {
    myPID.SetMode(AUTOMATIC);
    /* out the bounds */
    Serial.print("-- MOTOR ACTIVE realoutput: ");
    Serial.print(realOutput);
    Serial.print(" -- ");
    digitalWrite(enablePin, HIGH);
  }

  /*
    // frontend
    if (millis()>serialTime) {
      SerialReceive();
      SerialSend();
      serialTime+=500;
    }
  */


  /* debug logging */
  Serial.print("Relative angle: ");
  Serial.print(relativeAngle, 2);
  Serial.print(" -- PID output: ");
  Serial.println(pidOutput);
  //Serial.print(" -- proportional: ");
  //Serial.print(-- integral: ## -- derivative: ##")
  delay(100);

}
