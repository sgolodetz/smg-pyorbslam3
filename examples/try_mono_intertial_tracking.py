from smg.pyorbslam3 import *


def main():
    imu_point: IMUPoint = IMUPoint(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    imu_points: IMUPoints = IMUPoints()
    imu_points.append(imu_point)
    imu_points.append(imu_point)
    print(len(imu_points))
    temp(imu_points)
    print(len(imu_points))
    print(list(imu_points))
    print(imu_points)
    temp(imu_points)


if __name__ == "__main__":
    main()
