#include <Servo.h>

#define SENSOR_INPUT A0
#define SERVO_OUTPUT 3

#define SERVO_MAX 180        // max servo angle [deg]
#define SERVO_MIN 0          // min servo angle [deg]
const float FLAT_ANGLE = 90; // CALIBRATED, servo angle [deg] for flat beam

#define SENSOR_OFFSET                                                          \
  65.5 // CALIBRATED, distance from sensor to center of beam [cm]
const float MIN_DISTANCE_SENSOR = 10.0;

const float DESIRED_POSITION = 0.0;

// const float KP = 4;
// const float KD = 500;

const float KP = 5;
const float KD = 500;
const float KI = .01;

Servo beamservo;
float time;
float period = 60.0;     // control loop period [ms]
float position;          // ball position [cm]
float posavg = 0.0;      // slow average ball position [cm]
float posfiltered = 0.0; // filtered ball position [cm]

float error = 0.0;
float error_derived = .0; // tracked for lpf for D controller
float error_integral = .0;

float angleout = FLAT_ANGLE;
float clamped_angleout = angleout; // angle to write to servo motor

int serialbyte = -1; // for controlling angle from Serial input

void setup() {
  // servo motor
  beamservo.attach(SERVO_OUTPUT);
  beamservo.write(angleout);

  // serial port for plotting/debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  Serial.println("serial open.");
  pinMode(SENSOR_INPUT, INPUT);
  time = millis();
}

void loop() {
  if (millis() > time + period) // once every period do...
  {
    time = millis();

    /* sample system state (ball position) */
    position = get_dist(100) - SENSOR_OFFSET;
    posavg = posavg * 0.99 + position * 0.01;
    posfiltered = posfiltered * 0.8 + position * 0.2;

    float prev_error = error;
    error = DESIRED_POSITION - posfiltered;

    float prev_error_derived = error_derived;
    error_derived = (error - prev_error) / period;
    float error_derived_lpf = prev_error_derived * .6 + error_derived * .4;

    error_integral = error_integral + period * error;

    float P = KP * error;
    float I = KI / time * error_integral;
    float D = KD * error_derived_lpf;
    float PID = FLAT_ANGLE + P + I + D;

    // Some issue with this anti windup?
    float clamped_PID = clamp(PID, 180, 0);

    float delta = PID - clamped_PID;

    angleout = PID + delta;

    beamservo.write(angleout);

    /* print results formatted for Arduino Serial Plotter */
    printstate(position, posfiltered, P, I, D, PID, angleout);
  }
}
/*
float get_angle_corr_using_serial {
// adjust beam angle from user input
    serialbyte = Serial.read();
    switch (serialbyte)
    {
    case '0':
      return 90;
      break;
    case '1':
      return 1;
      break;
    case '2':
      return - 1;
      break;
    default:
      // statements
      break;
    }
    }
*/

/* return average of n consecutive position sensor readings [cm] */
float get_dist(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum = sum + analogRead(SENSOR_INPUT);
  }
  float adc = sum / n;
  float distance_cm =
      17569.7 *
      pow(adc, -1.2062); // TODO check vs datasheet for gp2y0a21yk0f sensor

  return (distance_cm > MIN_DISTANCE_SENSOR) ? distance_cm
                                             : MIN_DISTANCE_SENSOR;
}

/* clamp value between max and min */
float clamp(float value, float max, float min) {
  float clamped = value;
  if (value > max) {
    clamped = max;
  }
  if (clamped < min) {
    clamped = min;
  }

  return clamped;
}

void printstate(float position, float posfiltered, float P, float I, float D,
                float PID, float angleout) {

  /* print system and controller state */
  Serial.print("d:");
  Serial.print(position);
  Serial.print(",");
  Serial.print("df:");
  Serial.print(posfiltered);
  Serial.print(",");
  Serial.print("P:");
  Serial.print(P);
  Serial.print(",");
  Serial.print("I:");
  Serial.print(I);
  Serial.print(",");
  Serial.print("D:");
  Serial.print(D);
  Serial.print(",");
  Serial.print("PID:");
  Serial.print(PID);
  Serial.print(",");
  Serial.print("u:");
  Serial.print(angleout);
  Serial.print("\n");
  // Serial.print("P:");
  // Serial.print(_fill in here_); // proportional part of control signal
  // Serial.print(",");d
}
