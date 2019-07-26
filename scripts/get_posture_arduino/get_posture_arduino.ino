#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <TimerOne.h>

LSM9DS1 imu;
#define LSM9DS1_M 0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6A // Would be 0x6A if SDO_AG is LOW

// after start processing, sample sensor data for "NUM_OF_SAMPLES_FOR_INIT" times to cancel native noise of the sensor
#define NUM_OF_SAMPLES_FOR_INIT 200
// parameters to contain native noise of the sensor to be canceled
float offset_gx = 0;
float offset_gy = 0;
float offset_gz = 0;

// define sampling rate (Hz)
#define SAMPLING_RATE 100

// flag parameter to activate timer interruption
volatile int interrupt_flag = 1;


int ros_flag = 1; //flag==1 then use for ROS, flag==0 then use for debug
int noise_filtering_flag = 0; //ジャイロのスパイクノイズをフィルタする場合は１にする。

// parameter to contain time to measure spent time of a control loop
int previous_time = 0;
int present_time = 0;
int passed_time = 0;

// parameter to contain sensor value
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ, roll, pitch, heading, ACCroll, ACCpitch;
float last_gyroX = 0, last_gyroY = 0, last_gyroZ = 0, filtered_gyroX = 0, filtered_gyroY = 0, filtered_gyroZ = 0;



void setup() {
  //  start serial communication
  Serial.begin(115200);

  //  start sensor LSM9DS1 processing (refer sample code of the library)
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

  //  flush serial input buffer
  while (Serial.available() > 0) {
    Serial.read();
  }

  Timer1.initialize(1000000 / SAMPLING_RATE); //interrupt per 10000 micro seconds(10 msec)
  Timer1.attachInterrupt(interrupt_function);
}

void loop() {

  //  execute reading sensor process only if the interrupt flag is on
  if (interrupt_flag == 1) {
    //  get sensor data
    get_IMU_data();

    //  get posture data from complementary filter
    get_posture_complementary_filter();

    //  turn off the interruption flag
    interrupt_flag = 0;
    //    print_time();
  }


  //  (for dubugging) if it is not connected to ROS node
  if (flag == 0) {

    //    Serial.print(roll);
    //    Serial.print(',');
    //    Serial.print(pitch);
    //    Serial.print(',');
    //    Serial.println(heading);


    //    Serial.print(accelX);
    //    Serial.print(',');
    //    Serial.print(accelY);
    //    Serial.print(',');
    //    Serial.println(accelZ);


    //    Serial.print(gyroX);
    //    Serial.print(',');
    //    Serial.print(gyroY);
    //    Serial.print(',');
    //    Serial.println(gyroZ);
    //
    //    Serial.print(filtered_gyroX);
    //    Serial.print(',');
    //    Serial.print(filtered_gyroY);
    //    Serial.print(',');
    //    Serial.println(filtered_gyroZ);

    //    print_time();
  }



  //  if it is connected to ROS node to send IMU data to inverted pendulum
  if (ros_flag == 1) {

    //  receive message trigger from the connected ROS node 
    if (Serial.available() > 0) {
      while (Serial.available() > 0) {
        //  flush message trigger
        Serial.read();
      }

      //  send IMU data
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
}


//  function when timer interrupt has occured
//  just turn on the interruption flag, so that the IMU data read process will be executed
//  it could be implemented as read & process IMU data in this function, but it wouldn't work.
//  timer for interruption and I2C communication may be in conflict...? 
void interrupt_function() {
  interrupt_flag = 1;
}


// print the passed time per loop if you need
void print_time() {
  present_time = micros();
  passed_time = present_time - previous_time;
  Serial.println(passed_time);
  previous_time = present_time;
}
