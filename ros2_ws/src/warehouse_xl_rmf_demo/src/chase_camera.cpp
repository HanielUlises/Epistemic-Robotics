// Copyright 2026 Haniel Ulises
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Drives the Gazebo GUI camera so that it follows a robot.
//
// Gazebo Classic offers three ways to move that camera and two of them do not
// work here. The `<gui><camera><pose>` element in the world file is read once
// at load and cannot be revisited. `gz camera -c user_camera -f <model>`
// advertises on `~/user_camera/cmd` and then waits for a subscriber before it
// publishes; nothing subscribes to that topic, because it belongs to camera
// sensors and the GUI camera is not one, so the command blocks for ever and
// takes whatever script invoked it with it. What does work is
// `~/user_camera/joy_pose`, which the GUI camera subscribes to and which sets
// its pose outright.
//
// So the pose is computed here, at a fixed rate, from the robot's own pose as
// the server publishes it on `~/pose/info`. No ROS is involved: both
// topics are Gazebo transport, and a node that talks to the simulator about
// where to put a camera has no business on the robot's control graph.
//
// The camera trails the robot by `distance` metres along the robot's own
// heading, at `height` above the floor, and looks at a point on the robot's
// mast. Two details matter more than the numbers:
//
//   Smoothing. Following the heading exactly puts the camera through the same
//   rotation as the robot, and this robot turns in place at every waypoint.
//   The target pose is filtered towards and not jumped to, with a time
//   constant of a few tenths of a second, which lets the robot rotate within
//   the frame while the camera comes round after it.
//
//   A clock, and not the pose stream. The filter steps at a fixed rate and the
//   callback only records where the robot is. Gazebo publishes `~/pose/info`
//   for models whose pose has changed, so a robot that stops leaves the stream
//   and a camera driven from the callback stops with it. A robot stops by
//   turning in place to face its site, which is where the camera is furthest
//   from its target, so the first version froze mid-swing and left the robot a
//   speck at the end of an aisle.
//
//   Travel, not heading. The camera trails the direction the robot has been
//   moving in, held while it is stopped, and not the direction it faces. An
//   aisle is a corridor and a robot faces its site across the corridor, so
//   trailing the heading puts the camera inside a rack for the whole of the
//   sequence the film is about.
//
//   Yaw is filtered on the circle and not on the line. Interpolating 179
//   degrees towards -179 the short way is two degrees; the long way is 358,
//   and the camera swings all the way round the robot to arrive where it
//   started.
//
// The camera aims at a point `look` metres up the robot, so `height - look`
// over `distance` fixes the tilt, and the tilt fixes where in the frame the
// robot sits. Measured against this capture, the canvas is 1655x977 and the
// robot moves 2.6 per cent of the frame height per degree of tilt: at 4.5
// degrees it sits at 87 per cent and drops out of a 16:9 crop as soon as it
// turns, at 16 degrees it sits just below the middle. The defaults below are
// the second of those.
//
//     chase_camera --model r2 --distance 2.6 --height 1.25 --look 0.30

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

namespace
{

std::atomic<bool> running{true};

struct Config
{
  std::string model = "r2";
  double distance = 2.6;   // metres behind the robot
  double height = 1.25;    // metres above the floor
  double look = 0.30;      // height on the robot the camera aims at
  double tau = 0.45;       // seconds; the position filter's time constant
  double tau_yaw = 1.2;    // seconds; the travel-direction filter
  double lead = 0.0;       // metres ahead of the robot to aim
  double rate = 25.0;      // filter steps and camera poses per second
};

Config config;
gazebo::transport::PublisherPtr camera_pub;

std::mutex lock;
bool have_robot = false;          // a pose for the model has been seen
double rob_x = 0.0, rob_y = 0.0, rob_yaw = 0.0;

bool have_state = false;          // the camera has been placed
double cam_x = 0.0, cam_y = 0.0, cam_z = 0.0, cam_yaw = 0.0;
double prev_x = 0.0, prev_y = 0.0;   // the robot, one step ago
double vel_x = 0.0, vel_y = 0.0;     // its filtered velocity
long published = 0;

double wrap(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

// Interpolate an angle the short way round. A plain lerp between two angles
// either side of the cut takes the long way, which is a full revolution of
// the camera for a robot that did not turn at all.
double blend_angle(double from, double to, double alpha)
{
  return wrap(from + alpha * wrap(to - from));
}

double yaw_of(const gazebo::msgs::Pose & pose)
{
  const auto & q = pose.orientation();
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void on_poses(ConstPosesStampedPtr & msg)
{
  for (int i = 0; i < msg->pose_size(); ++i) {
    // The server reports a model's own pose under its name and its links
    // under "model::link". Only the first is the model, and matching on a
    // prefix would follow whichever link the world happened to list first.
    if (msg->pose(i).name() != config.model) {continue;}
    const auto & pose = msg->pose(i);
    std::lock_guard<std::mutex> held(lock);
    rob_x = pose.position().x();
    rob_y = pose.position().y();
    rob_yaw = yaw_of(pose);
    have_robot = true;
    return;
  }
}

// One step of the filter, at a fixed rate, and not one step per pose message.
//
// The two are not the same thing, and the difference is the whole of a failed
// recording. Gazebo publishes `~/pose/info` for models whose pose has changed,
// so a robot that stops leaves the stream, and a camera driven from the
// callback stops with it -- wherever it had got to. A robot stops by turning
// in place to face its site, which is the point in the trajectory at which the
// camera is furthest from its target, so the shot froze mid-swing and the
// robot was left a speck at the end of an aisle. Running the filter on a clock
// converges the camera whether the robot moves or not, and a stationary robot
// is exactly when a settled shot is wanted.
void step(double dt)
{
  double rx, ry, ryaw;
  {
    std::lock_guard<std::mutex> held(lock);
    if (!have_robot) {return;}
    rx = rob_x; ry = rob_y; ryaw = rob_yaw;
  }

  if (!have_state) {
    cam_yaw = ryaw;
    cam_x = rx - config.distance * std::cos(cam_yaw);
    cam_y = ry - config.distance * std::sin(cam_yaw);
    cam_z = config.height;
    prev_x = rx;
    prev_y = ry;
    have_state = true;
    return;
  }

  // The camera trails along the direction the robot has been travelling, and
  // holds that direction while the robot is stopped. This is what keeps the
  // camera out of the shelving, and the reason is geometric.
  //
  // An aisle is a corridor, and a robot arrives at a site by driving down the
  // corridor and then turning in place to face the site. Two and a half metres
  // behind a robot that has just turned through ninety degrees is inside the
  // rack the robot turned to face. Trailing the robot's heading walks the
  // camera into that rack, and slowing that filter only delays it: the robot
  // stands at the site for the whole epistemic sequence, so any filter on the
  // heading converges during the part of the run the film exists for.
  //
  // The direction of travel does not have that defect. It is undefined when
  // the robot is stopped, and holding the last value is right: the robot drove
  // through that space to get here, so there is nothing in it. The velocity is
  // filtered before it is used, since a robot turning in place creeps a
  // few millimetres a step and the direction of a creep is noise.
  const double vx = (rx - prev_x) / dt;
  const double vy = (ry - prev_y) / dt;
  prev_x = rx;
  prev_y = ry;
  const double beta = 1.0 - std::exp(-dt / 0.6);
  vel_x += beta * (vx - vel_x);
  vel_y += beta * (vy - vel_y);

  if (std::sqrt(vel_x * vel_x + vel_y * vel_y) > 0.08) {
    cam_yaw = blend_angle(cam_yaw, std::atan2(vel_y, vel_x),
                          1.0 - std::exp(-dt / config.tau_yaw));
  }

  const double want_x = rx - config.distance * std::cos(cam_yaw);
  const double want_y = ry - config.distance * std::sin(cam_yaw);

  const double alpha = 1.0 - std::exp(-dt / config.tau);
  cam_x += alpha * (want_x - cam_x);
  cam_y += alpha * (want_y - cam_y);
  cam_z += alpha * (config.height - cam_z);

  // Aim at the robot from where the camera actually is, not along the yaw it
  // is filtering towards. During a turn the two differ, and aiming along the
  // filtered yaw walks the robot out of the frame for as long as the filter
  // is catching up.
  const double aim_x = rx + config.lead * std::cos(ryaw);
  const double aim_y = ry + config.lead * std::sin(ryaw);
  const double dx = aim_x - cam_x;
  const double dy = aim_y - cam_y;
  const double flat = std::sqrt(dx * dx + dy * dy);
  const double aim_yaw = std::atan2(dy, dx);
  const double aim_pitch = std::atan2(cam_z - config.look, std::max(flat, 1e-3));

  gazebo::msgs::Pose out;
  gazebo::msgs::Set(out.mutable_position(), ignition::math::Vector3d(cam_x, cam_y, cam_z));
  gazebo::msgs::Set(
    out.mutable_orientation(),
    ignition::math::Quaterniond(0.0, aim_pitch, aim_yaw));
  camera_pub->Publish(out);
  ++published;
}

// The callback runs on a transport thread. An exception escaping it takes the
// whole process down without a message, which is how the first long recording
// came to hold a camera pointing at an aisle the robot had left.
void on_poses_guarded(ConstPosesStampedPtr & msg)
{
  try {
    on_poses(msg);
  } catch (const std::exception & e) {
    std::cerr << "chase_camera: " << e.what() << "\n" << std::flush;
  } catch (...) {
    std::cerr << "chase_camera: unknown exception in the pose callback\n"
              << std::flush;
  }
}

void stop(int) {running = false;}

}  // namespace

int main(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() {return (i + 1 < argc) ? std::string(argv[++i]) : std::string();};
    if (arg == "--model") {config.model = value();} else if (arg == "--distance") {
      config.distance = std::stod(value());
    } else if (arg == "--height") {config.height = std::stod(value());} else if (arg == "--look") {
      config.look = std::stod(value());
    } else if (arg == "--tau") {config.tau = std::stod(value());} else if (arg == "--lead") {
      config.lead = std::stod(value());
    } else if (arg == "--rate") {config.rate = std::stod(value());} else if (
      arg == "--tau-yaw") {config.tau_yaw = std::stod(value());} else if (arg == "--help") {
      std::cout << "chase_camera --model r2 --distance 2.6 --height 1.25 "
                << "--look 0.30 --tau 0.45 --tau-yaw 1.2 --rate 25\n";
      return 0;
    }
  }

  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);

  if (!gazebo::client::setup(argc, argv)) {
    std::cerr << "chase_camera: no Gazebo master; is gzserver running?\n";
    return 1;
  }

  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();

  // Advertised before the subscription so the first pose message already has
  // somewhere to go.
  camera_pub = node->Advertise<gazebo::msgs::Pose>("~/user_camera/joy_pose");
  auto sub = node->Subscribe("~/pose/info", on_poses_guarded);

  std::cerr << "chase_camera: following " << config.model << " at "
            << config.distance << " m, " << config.height << " m up, "
            << config.rate << " Hz\n" << std::flush;

  // A heartbeat, so a recording that comes out wrong says which of the two it
  // was: the camera stopped being driven, or it was driven to the wrong place.
  const auto period = std::chrono::duration<double>(1.0 / config.rate);
  auto next = std::chrono::steady_clock::now();
  long ticks = 0;
  while (running) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
    try {
      step(1.0 / config.rate);
    } catch (const std::exception & e) {
      std::cerr << "chase_camera: " << e.what() << "\n" << std::flush;
    }
    if (++ticks % (long)(config.rate * 30) == 0) {
      // The two positions, so a shot that comes out wrong can be read back
      // against the fleet's own report of where the robot was.
      std::lock_guard<std::mutex> held(lock);
      std::cerr << "chase_camera: " << published << " poses; robot ("
                << rob_x << ", " << rob_y << ") camera ("
                << cam_x << ", " << cam_y << ", " << cam_z << ") "
                << std::sqrt((rob_x - cam_x) * (rob_x - cam_x) +
                             (rob_y - cam_y) * (rob_y - cam_y))
                << " m back\n" << std::flush;
    }
  }

  sub.reset();
  gazebo::client::shutdown();
  return 0;
}
