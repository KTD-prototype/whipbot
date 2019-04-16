#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <TimerOne.h>

LSM9DS1 imu;
#define LSM9DS1_M 0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6A // Would be 0x6A if SDO_AG is LOW

// define the number of sample to get data to calibrate and sampling rate
#define NUM_OF_SAMPLES_FOR_INIT 200
#define SAMPLING_RATE 100

volatile int interrupt_flag = 1;
int flag = 1;

int previous_time = 0;
int present_time = 0;
int passed_time = 0;

float offset_gx = 0;
float offset_gy = 0;
float offset_gz = 0;

float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ, roll, pitch, heading, ACCroll, ACCpitch;



void setup() {
  Serial.begin(115200);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  setupGyro();
  setupAccel();
  setupMag();

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }

  // initial process to subtract gyro offset from measured data
  init_gyro_process();
  //  Serial.println("finished initialization !");
  //  Serial.println();

  while (Serial.available() > 0) {
    Serial.read();
  }

  Timer1.initialize(1000000 / SAMPLING_RATE); //interrupt per 10000 micro seconds(10 msec)
  Timer1.attachInterrupt(interrupt_function);
}

void loop() {
  if (interrupt_flag == 1) {
    get_IMU_data();
    get_posture_complementary_filter();
    interrupt_flag = 0;
    //    print_time();
  }

  if (flag == 1) {
    Serial.println(roll);
    Serial.println(pitch);
    Serial.println(heading);
    Serial.println(accelX);
    Serial.println(accelY);
    Serial.println(accelZ);
    Serial.println(gyroX);
    Serial.println(gyroY);
    Serial.println(gyroZ);
    flag = 0;
  }

  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      Serial.read();
      //      Serial.println("read!");
    }
    Serial.println(roll);
    Serial.println(pitch);
    Serial.println(heading);
    Serial.println(accelX);
    Serial.println(accelY);
    Serial.println(accelZ);
    Serial.println(gyroX);
    Serial.println(gyroY);
    Serial.println(gyroZ);
    //    print_time();
    Serial.flush();
  }
}


void interrupt_function() {
  interrupt_flag = 1;
}

void print_time() {
  present_time = micros();
  passed_time = present_time - previous_time;
  Serial.println(passed_time);
  previous_time = present_time;
}
