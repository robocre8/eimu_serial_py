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