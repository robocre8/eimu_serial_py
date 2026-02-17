

## Easy IMU (EIMU) Python Library
Python serial interface for the Easy IMU (EIMU).

> you can use it in your microcomputer robotics project (e.g Raspberry Pi, PC, etc.)

#

## Install
- you'll need to pip install the pyserial library
  ```shell
    pip3 install eimu-serial   //linux or mac
  ```
  ```shell
    pip install eimu-serial  //windows
  ```

#

## Uninstall
- you'll need to pip install the pyserial library
  ```shell
    pip3 uninstall eimu-serial   //linux or mac
  ```
  ```shell
    pip uninstall eimu-serial  //windows
  ```

#

## How to Use the Library
- Ensure you have the **`EIMU MODULE`** interfaced with your PC or microcomputer

- Ensure you have the **Easy IMU Module** is already calibrated and setup with the [eimu_setup_application](https://github.com/robocre8/eimu_setup_application)

- check the serial port the driver is connected to:
  ```shell
  ls /dev/ttyA*
  ```
  > you should see /dev/ttyACM0 or /dev/ttyACM1 and so on

- use the serial port in your code

- A simple way to get started is simply to try out the example code below

#

## Basic Library functions and usage

- connect to EIMU module
  > imu = EIMUSerialClient()
  >
  > imu.connect("port_name or port_path")

- clear imu, filter, e.t.c data buffer on the EIMU module
  > imu.clearDataBuffer() # returns bool -> success

- set imu reference frame -> NWU (0), ENU (1), NED (2) 
  > imu.setWorldFrameId(frame_id)

- get imu reference frame -> NWU (0), ENU (1), NED (2) 
  > imu.getWorldFrameId() # returns bool, int -> success, frame_id

- adjust filter gain
  > imu.setFilterGain(gain)

- read filter gain
  > imu.getFilterGain() # returns bool, float -> success, gain

- read all IMU data (orientation - RPY, linear acceleration, angular velocity)
  > imu.readImuData() # returns bool, tuple(size=9foats) -> success, (r, p, y, ax, ay, az, gx, gy, gz)

- read Oreintation - Quaterninos
  > imu.readQuat() # returns bool, tuple(size=4foats) -> success, (qw, qx, qy, qz)

- read Oreintation - RPY
  > imu.readRPY() # returns bool, tuple(size=3foats) -> success, (r, p, y)

- while these function above help communicate with the already configure EPMC module, more examples of advanced funtions usage for parameter tuning can be found in the [eimu_setup_application](https://github.com/robocre8/eimu_setup_application) source code

#

## example code
```python
from eimu_serial import EIMUSerialClient
import time
from math import pi

toRad = 2 * pi / 360
toDeg = 1 / toRad

imu = EIMUSerialClient()

def main():
  # 50Hz comm setup
  serial_port = '/dev/ttyACM0'
  serial_baudrate = 115200
  serial_timeout = 0.018 #value < 0.02 (for 50Hz comm)

  imu.connect(serial_port, serial_baudrate, serial_timeout)

  # success = imu.clearDataBuffer()

  # change the reference frame to ENU frame (0 - NWU,  1 - ENU,  2 - NED)
  imu.setWorldFrameId(1)

  # check the reference frame the eimu is working in (0 - NWU,  1 - ENU,  2 - NED)
  success, ref_frame_id = imu.getWorldFrameId()
  if success:
    if ref_frame_id == 0:
      print("Reference Frame is North-West-Up (NWU)")
    elif ref_frame_id == 1:
      print("Reference Frame is East-North-Up (ENU)")
    elif ref_frame_id == 2:
      print("Reference Frame is North-East-Down (NED)")
  else:
    print("Could not get world frame ID")

  prevTime = time.time()
  sampleTime = 0.02 #( 50Hz comm)

  while True:
    if time.time() - prevTime > sampleTime:
      # success, buffer = imu.readQuat() # qw, qx, qy, qz
      # if success:
      #   qw = buffer[0]
      #   qx = buffer[1]
      #   qy = buffer[2]
      #   qz = buffer[3]

      success, buffer = imu.readImuData() # r, p, y, ax, ay, az, gx, gy, gz
      if success:
        r = buffer[0]; p = buffer[1]; y = buffer[2]
        ax = buffer[3]; ay = buffer[4]; az = buffer[5]
        gx = buffer[6]; gy = buffer[7]; gz = buffer[8]

        print(f"r: {round(r*toDeg,2)}\tp: {round(p*toDeg,2)}\ty: {round(y*toDeg,2)}")
        print(f"ax: {ax}\tay: {ay}\taz: {az}")
        print(f"gx: {gx}\tgy: {gy}\tgz: {gz}")
        print()
      else:
        print("Error reading IMU data")
      prevTime = time.time()

if __name__ == "__main__":
  main()
```