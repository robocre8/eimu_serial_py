
## Easy IMU Python Serial Client Library
This library helps communicate with the already setup **`Easy IMU Module`** in your PC or microcomputer-based python projects, with the [eimu_setup_application](https://github.com/samuko-things-company/eimu_setup_application).

> you can use it in your microcomputer robotics project (e.g Raspberry Pi, PC, etc.)

A simple way to get started is simply to try out and follow the example code


## Dependencies
- you'll need to pip install the pyserial library
  ```shell
    pip3 install pyserial   //linux or mac
    pip install pyserial   //windows
  ```

## How to Use the Library
- Download (by clicking on the green Code button above) or clone the repo into your PC using **`git clone`**
> [!NOTE]  
> you can use this command if you want to clone the repo:
> 
>  ```git clone https://github.com/robocre8/eimu_serial_py.git```

- Ensure you have the **Easy IMU Module** is already calibrated.

- Connect the **Easy IMU Module** to your PC or microcomputer

- A simple way to get started is simply to try out and follow the example `read_rpy.py` code.

- You can copy the **`eimu_serial.py`** file into your python robotics project, import the library as shown in the example **`read_imu.py`** code, add it to your code, and start using it.


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

- read Linear Acceleration
  > imu.readLinearAcc() # returns bool, tuple(size=3foats) -> success, (ax, ay, az)

- read Gyro (Angular velocity)
  > imu.readGyro() # returns bool, tuple(size=3foats) -> success, (gx, gy, gz)