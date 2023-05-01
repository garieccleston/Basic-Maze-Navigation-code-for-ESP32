#include <Wire.h> 
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <stdbool.h>
#include <MPU6050_tockn.h>
#define _USE_MATH_DEFINES
#define I2C_SLAVE_ADDR 0x04
#define NUM_BYTES 8

LiquidCrystal lcd(14, 27, 19, 1, 3, 26);
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  { '1', '2', '3' },
  { '4', '5', '6' },
  { '7', '8', '9' },
  { '*', '0', '#' }
};

byte rowPins[ROWS] = { 15, 2, 0, 4 };
byte colPins[COLS] = { 16, 17, 5 };

float duration_us, distance_cm, reference_yaw, yaw_diff;

// variables to store the counts of each encoder, and the distance travelled by the EEEBot
long enc1_count, enc2_count, dist_travelled, reference_dist;

// variables to store motor speeds and servo angle
int left_speed, right_speed, servo_angle;

int left = 1, right = 2, forwards = 3, backwards = 4, stop = 5, adjust = 6;
// constants for motors & servo
const int MAX_MOTOR_SPEED = 255;
const int MIN_MOTOR_SPEED = -255;
const int SERVO_ANGLE_LEFT = 20;
const int SERVO_ANGLE_RIGHT = 160;
volatile int32_t position;
volatile uint8_t state;

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

long distanceTravelled(long encoder);
float readEncoderData();
MPU6050 mpu6050(Wire);

void setup() {
  lcd.begin(16, 2);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  move(stop);
  int i,c,c2,a,angle,tempangle,duration;
  char commandlist[1000];
  char durationlist[1000];
  memset(commandlist, 0, sizeof(1000));
  memset(commandlist, 0, sizeof(1000));
  char menucommand;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Entered command:");
  lcd.setCursor(0, 1);
  while (menucommand != '#') {  // breaks out of the loop and runs the entered commands
    while (i < 3) {
      char key = keypad.getKey();

      if (key) {
        lcd.setCursor(i, 1);
        lcd.print(key);
        if (i == 0) {
          commandlist[c] = key;  // saving command into arrays
          i = i + 1;
        } else if (i == 1) {
          durationlist[c] = key;
          i = i + 1;
        } else if (i == 2) {
          menucommand = key;
          i = i + 1;
        }
      }
    }
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.setCursor(1, 1);
    lcd.print(" ");
    lcd.setCursor(2, 1);
    lcd.print(" ");
    i = 0;
    c = c + 1;
  }
  c = c + 1;

  for (c2 = 0; c2 <= c; c2++) {  // iterates from 0 to c where c is the amount of commands entered
    switch (commandlist[c2]) {
      case '1':
        move(forwards);
        duration = (durationlist[c2] - '0');
        delay(250 * duration);
        move(adjust);
        delay(1000);
        break;
      case '2':
        move(backwards);
        duration = (durationlist[c2] - '0');
        delay(250 * duration);
        move(adjust);
        delay(1000);
        break;
      case '3':
        move(right);
        mpu6050.update();
        angle = 0;
        tempangle = mpu6050.getAngleZ();
        while (-angle < (tempangle + 85)) {
          mpu6050.update();
          angle = mpu6050.getAngleZ();
        }
        move(adjust);
        delay(1000);
        break;
      case '4':
        move(left);
        mpu6050.update();
        tempangle = mpu6050.getAngleZ();
        while (angle < (tempangle + 85)) {
          mpu6050.update();
          angle = mpu6050.getAngleZ();
        }
        move(adjust);
        delay(1000);
        break;
    }
  }
}

void move(int direction) {
  switch (direction) {
    case 1:  // turn left
      left_speed = -150;
      right_speed = 170;
      servo_angle = SERVO_ANGLE_LEFT;
      break;

    case 2:  // turn right
      left_speed = 170;
      right_speed = -150;
      servo_angle = SERVO_ANGLE_RIGHT;
      break;

    case 3:  // go forwards
      left_speed = 170;
      right_speed = 170;
      servo_angle = 85;
      break;

    case 4:  // go backwards
      left_speed = -170;
      right_speed = -170;
      servo_angle = 85;
      break;

    case 5:  // stop
      left_speed = 0;
      right_speed = 0;
      servo_angle = 85;
      break;

    case 6:  // steering adjust
      left_speed = 0;
      right_speed = 0;
      servo_angle = 85;
      break;
  }
  Wire.beginTransmission(I2C_SLAVE_ADDR);              // transmit to device #4
  Wire.write((byte)((left_speed & 0x0000FF00) >> 8));  // first byte of left_speed, containing bits 16 to 9
  Wire.write((byte)(left_speed & 0x000000FF));         // second byte of left_speed, containing the 8 LSB - bits 8 to 1

  Wire.write((byte)((right_speed & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(right_speed & 0x000000FF));

  Wire.write((byte)((servo_angle & 0x0000FF00) >> 8));  // first byte of y, containing bits 16 to 9
  Wire.write((byte)(servo_angle & 0x000000FF));
  Wire.endTransmission();
}

float readEncoderData()
{
  Wire.requestFrom(I2C_SLAVE_ADDR, 4);

  int16_t enc1_count;
  int16_t enc2_count;

  uint8_t enc1_count16_9 = Wire.read();   // receive bits 16 to 9 of y (one byte)
  uint8_t enc1_count8_1 = Wire.read();   // receive bits 8 to 1 of y (one byte)
  uint8_t enc2_count16_9 = Wire.read();   // receive bits 16 to 9 of z (one byte)
  uint8_t enc2_count8_1 = Wire.read();   // receive bits 8 to 1 of z (one byte)

  enc1_count = (enc1_count16_9 << 8) | enc1_count8_1; // combine the two bytes into a 16 bit number
  enc2_count = (enc2_count16_9 << 8) | enc2_count8_1;

  long encoder = ((enc1_count + enc2_count) / 2);

  return(encoder);
}
long distanceTravelled(long encoder)
{
  long distance = (1.570796326794 * encoder);
  return (distance);
}

