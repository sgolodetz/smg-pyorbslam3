#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
namespace py = pybind11;

#include <Python.h>

#pragma warning(disable:4244 4267 4305 4996)
#include "System.h"
#pragma warning(default:4244 4267 4305 4996)
using namespace ORB_SLAM3;

PYBIND11_MODULE(pyorbslam3, m)
{
  // CLASSES

  py::class_<System>(m, "System")
    .def(py::init<std::string, std::string, System::eSensor, bool>(), py::call_guard<py::gil_scoped_release>())
    .def("shutdown", &System::Shutdown, py::call_guard<py::gil_scoped_release>())
    .def("track_monocular", [](System& self, const cv::Mat3b& im, float timestamp) -> cv::Mat1d {
      return self.TrackMonocular(im, timestamp);
    }, py::call_guard<py::gil_scoped_release>())
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