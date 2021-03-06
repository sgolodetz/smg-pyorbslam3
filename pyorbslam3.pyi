from typing import Iterable, Iterator, Optional

from smg.pyopencv import CVMat1d, CVMat1f, CVMat3b


# CLASSES

class IMUPoint:
	def __init__(self, acc_x: float, acc_y: float, acc_z: float, ang_vel_x: float, ang_vel_y: float, ang_vel_z: float, timestamp: float): ...
	def __repr__(self) -> str: ...

class IMUPoints:
	def __init__(self, it: Optional[Iterable] = None): ...
	def __iter__(self) -> Iterator[IMUPoint]: ...
	def __len__(self) -> int: ...
	def __repr__(self) -> str: ...
	def append(self, x: IMUPoint) -> None: ...
	def clear(self) -> None: ...
	def insert(self, i: int, x: IMUPoint) -> None: ...
	def pop(self, i: Optional[int] = None) -> IMUPoint: ...

class System:
	def __init__(self, voc_file: str, settings_file: str, sensor: ESensor, use_viewer: bool): ...
	def shutdown(self) -> None: ...
	def track_monocular(self, im: CVMat3b, timestamp: float, v_imu_meas: IMUPoints = IMUPoints()) -> CVMat1d: ...
	def track_rgbd(self, im: CVMat3b, depthmap: CVMat1f, timestamp: float) -> CVMat1d: ...

# ENUMERATIONS

class ESensor(int):
	pass

MONOCULAR: ESensor
STEREO: ESensor
RGBD: ESensor
