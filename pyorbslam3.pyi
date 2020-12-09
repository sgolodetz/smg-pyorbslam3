from smg.pyopencv import CVMat1d, CVMat1f, CVMat3b


# CLASSES

class System:
	def __init__(self, voc_file: str, settings_file: str, sensor: ESensor, use_viewer: bool): ...
	def shutdown(self) -> None: ...
	def track_monocular(self, im: CVMat3b, timestamp: float) -> CVMat1d: ...
	def track_rgbd(self, im: CVMat3b, depthmap: CVMat1f, timestamp: float) -> CVMat1d: ...

# ENUMERATIONS

class ESensor(int):
	pass

MONOCULAR: ESensor
STEREO: ESensor
RGBD: ESensor
