#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <stdlib.h>
#include <string.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo setup
const byte X_SERVO_CHANNEL = 0;
const byte Y_SERVO_CHANNEL = 1;
const float X_CENTER_DEG = 107.0;
const float Y_CENTER_DEG = 115.0;
const int SERVO_MIN = 110;
const int SERVO_MAX = 510;

// Movement limits and update timing
const float MAX_X_OFFSET_DEG = 18.0;
const float MAX_Y_OFFSET_DEG = 18.0;
const float MAX_SERVO_SPEED_DEG_PER_SEC = 180.0;
const unsigned long SERIAL_TIMEOUT_MS = 8000;
const unsigned long UPDATE_PERIOD_MS = 20;

float targetXOffset = 0.0;
float targetYOffset = 0.0;
float currentXOffset = 0.0;
float currentYOffset = 0.0;

unsigned long lastSerialMessageMs = 0;
unsigned long lastUpdateMs = 0;

char receiveBuffer[40];
byte receiveIndex = 0;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

float moveToward(float current, float target, float maximumStep) {
  if (target > current + maximumStep) {
    return current + maximumStep;
  }
  if (target < current - maximumStep) {
    return current - maximumStep;
  }
  return target;
}

int angleToPulse(float angleDegrees) {
  angleDegrees = clampFloat(angleDegrees, 0.0, 180.0);

  float pulse = SERVO_MIN
    + (angleDegrees / 180.0)
    * (SERVO_MAX - SERVO_MIN);

  return (int)(pulse + 0.5);
}

void writeServoAngles() {
  float xMin = X_CENTER_DEG - MAX_X_OFFSET_DEG;
  float xMax = X_CENTER_DEG + MAX_X_OFFSET_DEG;
  float yMin = Y_CENTER_DEG - MAX_Y_OFFSET_DEG;
  float yMax = Y_CENTER_DEG + MAX_Y_OFFSET_DEG;

  float xAngle = clampFloat(
    X_CENTER_DEG + currentXOffset,
    xMin,
    xMax
  );

  float yAngle = clampFloat(
    Y_CENTER_DEG + currentYOffset,
    yMin,
    yMax
  );

  pwm.setPWM(X_SERVO_CHANNEL, 0, angleToPulse(xAngle));
  pwm.setPWM(Y_SERVO_CHANNEL, 0, angleToPulse(yAngle));
}

void printAcceptedCommand() {
  Serial.print("ACK X=");
  Serial.print(targetXOffset, 3);
  Serial.print(" Y=");
  Serial.println(targetYOffset, 3);
}

void processCompleteMessage() {
  receiveBuffer[receiveIndex] = '\0';
  char* comma = strchr(receiveBuffer, ',');

  if (comma == NULL) {
    Serial.print("ERROR missing comma: ");
    Serial.println(receiveBuffer);
    receiveIndex = 0;
    return;
  }

  *comma = '\0';

  char* xText = receiveBuffer;
  char* yText = comma + 1;
  char* xEnd;
  char* yEnd;

  float receivedX = strtod(xText, &xEnd);
  float receivedY = strtod(yText, &yEnd);

  if (xEnd == xText || yEnd == yText) {
    Serial.println("ERROR invalid number");
    receiveIndex = 0;
    return;
  }

  targetXOffset = clampFloat(
    receivedX,
    -MAX_X_OFFSET_DEG,
    MAX_X_OFFSET_DEG
  );

  targetYOffset = clampFloat(
    receivedY,
    -MAX_Y_OFFSET_DEG,
    MAX_Y_OFFSET_DEG
  );

  lastSerialMessageMs = millis();

  // Uncomment when debugging serial commands.
  // printAcceptedCommand();

  receiveIndex = 0;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char incoming = Serial.read();

    if (incoming == '\n') {
      processCompleteMessage();
    }
    else if (incoming != '\r') {
      if (receiveIndex < sizeof(receiveBuffer) - 1) {
        receiveBuffer[receiveIndex] = incoming;
        receiveIndex++;
      }
      else {
        receiveIndex = 0;
        Serial.println("ERROR message too long");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(20);

  currentXOffset = 0.0;
  currentYOffset = 0.0;
  targetXOffset = 0.0;
  targetYOffset = 0.0;

  writeServoAngles();

  lastSerialMessageMs = millis();
  lastUpdateMs = millis();

  delay(500);

  Serial.println("READY PCA9685");
  Serial.print("X center: ");
  Serial.println(X_CENTER_DEG);
  Serial.print("Y center: ");
  Serial.println(Y_CENTER_DEG);
  Serial.print("Max X movement: +/- ");
  Serial.println(MAX_X_OFFSET_DEG);
  Serial.print("Max Y movement: +/- ");
  Serial.println(MAX_Y_OFFSET_DEG);
}

void loop() {
  readSerialCommands();

  unsigned long now = millis();

  if (now - lastSerialMessageMs > SERIAL_TIMEOUT_MS) {
    targetXOffset = 0.0;
    targetYOffset = 0.0;
  }

  if (now - lastUpdateMs >= UPDATE_PERIOD_MS) {
    float dtSeconds = (now - lastUpdateMs) / 1000.0;
    lastUpdateMs = now;

    if (dtSeconds > 0.1) {
      dtSeconds = 0.1;
    }

    float maximumStep = MAX_SERVO_SPEED_DEG_PER_SEC * dtSeconds;

    currentXOffset = moveToward(
      currentXOffset,
      targetXOffset,
      maximumStep
    );

    currentYOffset = moveToward(
      currentYOffset,
      targetYOffset,
      maximumStep
    );

    writeServoAngles();
  }
}
