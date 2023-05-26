from dataclasses import dataclass

@dataclass
class Quat_array:
    x: float
    y: float
    z: float
    w: float

@dataclass
class Acc_array:
    x: float
    y: float
    z: float

@dataclass
class Gyro_array:
    x: float
    y: float
    z: float

@dataclass
class Mag_array:
    x: float
    y: float
    z: float

@dataclass
class IMU:
    time: float
    sensor_name: str
    sensor_id: int
    orientation: Quat_array
    accelerometer: Acc_array
    gyroscope: Acc_array
    magnetometer: Acc_array

