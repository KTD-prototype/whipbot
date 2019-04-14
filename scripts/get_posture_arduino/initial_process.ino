float sum_gx = 0;
float sum_gy = 0;
float sum_gz = 0;


void init_gyro_process() {
  for (int i = 0; i < NUM_OF_SAMPLES_FOR_INIT; i++) {
    // wait for new data available
    while (imu.gyroAvailable() != 1) {
    }
    imu.readGyro();
    sum_gx += imu.gx;
    sum_gy += imu.gy;
    sum_gz += imu.gz;
  }

  offset_gx = sum_gx / NUM_OF_SAMPLES_FOR_INIT;
  offset_gy = sum_gy / NUM_OF_SAMPLES_FOR_INIT;
  offset_gz = sum_gz / NUM_OF_SAMPLES_FOR_INIT;
}
