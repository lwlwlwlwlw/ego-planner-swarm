#!/usr/bin/python3
"""Exercise the native trajectory server without starting ROS or a simulation."""

from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
PROGRAM = r'''
#define main trajectory_server_main
#include "src/planner/plan_manage/src/traj_server.cpp"
#undef main
#include <cmath>
#include <string>

int main(int argc, char **argv) {
  if (argc != 2) return 10;
  Eigen::MatrixXd points(3, 6);
  points << 0., 1., 2., 3., 4., 5.,
            0., 1., 2., 3., 4., 5.,
            2., 2., 2., 2., 2., 2.;
  traj_.emplace_back(points, 3, 1.);
  traj_duration_ = traj_[0].getTimeSum();
  time_forward_ = 0.;
  last_yaw_ = last_yaw_dot_ = 0.;
  ros::Time previous(20, 0);
  Eigen::Vector3d position = traj_[0].evaluateDeBoorT(0.);
  std::string mode(argv[1]);
  if (mode == "first") {
    ros::Time current = previous;
    auto yaw = calculate_yaw(0., position, current, previous);
    if (!std::isfinite(yaw.first) || !std::isfinite(yaw.second)) return 1;
    if (yaw.second != 0.) return 2;
  } else {
    time_forward_ = 1.;
    ros::Time current(20, 10000000);
    auto yaw = calculate_yaw(0., position, current, previous);
    if (!std::isfinite(yaw.first) || !std::isfinite(yaw.second)) return 3;
    if (yaw.first <= 0. || yaw.first > 3.1415926 * .01) return 4;
    previous = current;
    const double before = last_yaw_;
    if (mode == "backward") current = ros::Time(20, 0);
    else if (mode != "repeated") return 11;
    yaw = calculate_yaw(0., position, current, previous);
    if (!std::isfinite(yaw.first) || !std::isfinite(yaw.second)) return 5;
    if (yaw.first != before || yaw.second != 0.) return 6;
    previous = current;
  }
  for (int tick = 1; tick <= 30; ++tick) {
    ros::Time current = previous + ros::Duration(.01);
    auto yaw = calculate_yaw(0., position, current, previous);
    if (!std::isfinite(yaw.first) || !std::isfinite(yaw.second)) return 7;
    if (std::abs(yaw.second) > 3.1415926) return 8;
    previous = current;
  }
}
'''


class TrajectoryYawTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="ego-traj-yaw-")
        cls.addClassCleanup(cls.directory.cleanup)
        directory = Path(cls.directory.name)
        source = directory / "test.cpp"
        source.write_text(PROGRAM)
        cls.executable = directory / "test-yaw"
        subprocess.run([
            "g++", "-std=c++14", "-I", str(ROOT), "-I/usr/include/eigen3",
            "-I/opt/ros/noetic/include", "-I", str(ROOT / "devel/include"),
            "-I", str(ROOT / "src/planner/bspline_opt/include"),
            str(source), "-L", str(ROOT / "devel/lib"),
            "-Wl,-rpath," + str(ROOT / "devel/lib"), "-lbspline_opt",
            "-L/opt/ros/noetic/lib", "-Wl,-rpath,/opt/ros/noetic/lib",
            "-lroscpp", "-lrosconsole", "-lroscpp_serialization",
            "-lrostime", "-lcpp_common", "-o", str(cls.executable),
        ], check=True, capture_output=True, text=True)

    def check_case(self, mode):
        result = subprocess.run([str(self.executable), mode], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0,
                         f"{mode}: native yaw calculation failed ({result.returncode})\n{result.stderr}")

    def test_first_tick_and_following_samples_are_finite(self):
        self.check_case("first")

    def test_repeated_clock_does_not_corrupt_following_samples(self):
        self.check_case("repeated")

    def test_backward_clock_keeps_heading_and_finite_following_samples(self):
        self.check_case("backward")


if __name__ == "__main__":
    unittest.main()
