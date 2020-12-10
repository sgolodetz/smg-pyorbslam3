#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
namespace py = pybind11;

#include <Python.h>

#include <sstream>

#pragma warning(disable:4244 4267 4305 4996)
#include "System.h"
#pragma warning(default:4244 4267 4305 4996)
using namespace ORB_SLAM3;

PYBIND11_MAKE_OPAQUE(std::vector<IMU::Point>);

std::string to_string(const IMU::Point& self)
{
  return std::string("IMUPoint(") +
    std::to_string(self.a.x) + ", " + std::to_string(self.a.y) + ", " + std::to_string(self.a.z) + ", " +
    std::to_string(self.w.x) + ", " + std::to_string(self.w.y) + ", " + std::to_string(self.w.z) + ", " +
    std::to_string(self.t) +
  ")";
}

std::string to_string(const std::vector<IMU::Point>& self)
{
  std::ostringstream oss;
  oss << "IMUPoints([\n";
  std::string sep = "";
  for(const IMU::Point& p: self)
  {
    oss << sep << "  " << to_string(p);
    sep = ",\n";
  }
  oss << "\n])";
  return oss.str();
}

PYBIND11_MODULE(pyorbslam3, m)
{
  // CLASSES

  py::class_<IMU::Point>(m, "IMUPoint")
    .def(
      py::init<const float&, const float&, const float&, const float&, const float&, const float&, const double&>(),
      py::call_guard<py::gil_scoped_release>()
    )
    .def("__repr__", (std::string(*)(const IMU::Point&))&to_string, py::call_guard<py::gil_scoped_release>())
  ;

  py::bind_vector<std::vector<IMU::Point>>(m, "IMUPoints")
    .def("__repr__", (std::string(*)(const std::vector<IMU::Point>&))&to_string, py::call_guard<py::gil_scoped_release>())
  ;

  py::class_<System>(m, "System")
    .def(py::init<std::string, std::string, System::eSensor, bool>(), py::call_guard<py::gil_scoped_release>())
    .def("shutdown", &System::Shutdown, py::call_guard<py::gil_scoped_release>())
    .def(
      "track_monocular",
      [](System& self, const cv::Mat3b& im, float timestamp, const std::vector<IMU::Point>& vImuMeas) -> cv::Mat1d
      {
        return self.TrackMonocular(im, timestamp, vImuMeas);
      },
      py::arg("im"), py::arg("timestamp"), py::arg("vImuMeas") = std::vector<IMU::Point>(),
      py::call_guard<py::gil_scoped_release>()
    )
    .def("track_rgbd", [](System& self, const cv::Mat3b& im, const cv::Mat1f& depthmap, float timestamp) -> cv::Mat1d {
      return self.TrackRGBD(im, depthmap, timestamp);
    }, py::call_guard<py::gil_scoped_release>())
  ;

  // ENUMERATIONS

  py::enum_<System::eSensor>(m, "ESensor")
    .value("MONOCULAR", System::MONOCULAR)
    .value("STEREO", System::STEREO)
    .value("RGBD", System::RGBD)
    .export_values()
  ;
}
