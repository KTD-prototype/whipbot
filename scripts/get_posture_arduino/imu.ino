float k, g;
float a = 0.7;
#define C 1

void get_IMU_data() {
  imu.readGyro();
  delayMicroseconds(100);
  imu.readAccel();
  delayMicroseconds(100);
  gyroX = imu.calcGyro(imu.gx - offset_gx);
  gyroY = imu.calcGyro(imu.gy - offset_gy);
  gyroZ = imu.calcGyro(imu.gz - offset_gz);
  accelX = imu.calcAccel(imu.ax);
  accelY = imu.calcAccel(imu.ay);
  accelZ = imu.calcAccel(imu.az);
  magX = imu.calcMag(imu.mx);
  magY = imu.calcMag(imu.my);
  magZ = imu.calcMag(imu.mz);


  //ジャイロの値にスパイクノイズが入るため、フィルタする。
  if (noise_filtering_flag == 1) {
    filtered_gyroX = a * last_gyroX + (1 - a) * gyroX;
    filtered_gyroY = a * last_gyroY + (1 - a) * gyroY;
    filtered_gyroZ = a * last_gyroZ + (1 - a) * gyroZ;
  }
  last_gyroX = filtered_gyroX;
  last_gyroY = filtered_gyroY;
  last_gyroZ = filtered_gyroZ;
}


void get_posture_complementary_filter() {
  g = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
  //  k = 0.05 - (g - 1) / 100;
  //  if (k < 0) {
  //    k = 0;
  //  }
  k = 0.1 * pow(65536, -1 * (pow((1 - g), 2) / C));
  ACCroll = 0.9 * ACCroll + 0.1 * (atan2(accelY,  accelZ) * 180 / M_PI);
  ACCpitch = 0.9 * ACCpitch + 0.1 * (atan2(accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / M_PI);

  //calculate pitch/roll/yaw by gyrosensor (About pitch & roll, using Complementary Filter)
  //(とりあえずピッチのみ）角速度を相補フィルタリング後の値から再計算する。
  roll = (1 - k) * (roll + gyroX  / SAMPLING_RATE) + k * ACCroll;
  pitch = (1 - k) * (pitch + (-1 * gyroY) / SAMPLING_RATE) + k * ACCpitch; //ピッチの角速度データが正負逆で入っているようだ。

  if (abs(gyroZ) < 0.05) {
    heading = heading;
  }
  else heading = heading + gyroZ / SAMPLING_RATE;

  if (heading > 180) {
    heading = -360 + heading;
  }
  else if (heading < -180) {
    heading = 360 + heading;
  }
}
