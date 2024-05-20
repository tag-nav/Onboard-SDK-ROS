/** @file advanced_sensing_node.cpp
 *  @version 4.0
 *  @date May 2020
 *
 *  @brief sample node of flight control.
 *
 *  @Copyright (c) 2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

//INCLUDE
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

#include <dji_osdk_ros/SetJoystickMode.h>
#include <dji_osdk_ros/JoystickAction.h>
#include <dji_osdk_ros/dji_vehicle_node.h>
#include <cmath>

struct quaternion {double q0, q1, q2, q3;};
struct EulerAngles {double x, y, z;};
struct vector3f {float x, y, z;};

//CODE
using namespace dji_osdk_ros;

geometry_msgs::QuaternionStamped attitude_data_;
sensor_msgs::NavSatFix rtk_position_;
sensor_msgs::NavSatFix gps_position_;
std_msgs::Float32 height_above_takeoff_;

ros::ServiceClient task_control_client;
ros::ServiceClient set_joystick_mode_client;
ros::ServiceClient joystick_action_client;

ros::Subscriber rtkPositionSub;
ros::Subscriber gpsPositionSub;
ros::Subscriber attitudeSub;
ros::Subscriber heightSub;

void attitudeSubCallback(const geometry_msgs::QuaternionStampedConstPtr& attitudeData)
{
  attitude_data_ = *attitudeData;
}

void rtkPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& rtkPosition)
{
  rtk_position_ = *rtkPosition;
}

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{
  gps_position_ = *gpsPosition;
}

void heightSubCallback(const std_msgs::Float32::ConstPtr& heightAboveTakeoff)
{
  height_above_takeoff_ = *heightAboveTakeoff;
}

bool moveByPosOffset(FlightTaskControl& task, const JoystickCommand &offsetDesired,
                     float posThresholdInM, float yawThresholdInDeg, float speedFactor);

void returnToHome(FlightTaskControl& task, const JoystickCommand &offsetDesired,
                     float posThresholdInM, float yawThresholdInDeg, float speedFactor, float32_t homeRTKLat, float32_t homeRTKLong, float32_t homeRTKHeight);

void goHome(FlightTaskControl& task);

void velocityAndYawRateCtrl(const JoystickCommand &offsetDesired, uint32_t timeMs);

float32_t vectorNorm(vector3f v)
{
  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

template <typename Type>
int signOfData(Type type)
{
  return type < 0 ? -1 : 1;
}

void horizCommandLimit(float speedFactor, float& commandX,
                                        float& commandY)
{
  // if (fabs(commandX) > speedFactor)
  //   commandX = signOfData<float>(commandX) * speedFactor;
  // if (fabs(commandY) > speedFactor)
  //   commandY = signOfData<float>(commandY) * speedFactor;
  commandX = (fabs(commandX) < speedFactor) ? commandX : speedFactor;
  commandY = (fabs(commandY) < speedFactor) ? commandY : speedFactor;
}

void vertiCommandLimit(float zSpeedFactor, float& commandZ)
{
  commandZ = (fabs(commandZ) < zSpeedFactor) ? commandZ : zSpeedFactor;
}

vector3f localOffsetFromRTKOffset(
  const float32_t& targetLat, const float32_t& targetLong, const float32_t& originLat,
  const float32_t& originLong, const float32_t& targetHeight, const float32_t& originHeight)
{
  vector3f deltaNed;
  double deltaLon = targetLong - originLong;
  double deltaLat = targetLat -  originLat;
  deltaNed.x = deltaLat * C_EARTH;
  deltaNed.y = deltaLon * C_EARTH * cos(targetLat);
  deltaNed.z = targetHeight - originHeight;
  return deltaNed;
}

vector3f offsetRemainingFromRTKOffset(
  const float32_t& targetLat, const float32_t& targetLong, const float32_t& originLat,
  const float32_t& originLong, const float32_t& targetHeight, const float32_t& originHeight)
{
  vector3f deltaNed;
  double deltaLon = targetLong - originLong;
  double deltaLat = targetLat -  originLat;
  deltaNed.x = deltaLat * C_EARTH;
  deltaNed.y = deltaLon * C_EARTH * cos(targetLat);
  deltaNed.z = targetHeight - originHeight;
  return deltaNed;
}

 EulerAngles quaternionToEulerAngle(const quaternion& quat)
  {
    EulerAngles eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
  }

 vector3f vector3FSub(const JoystickCommand& vectorA, const vector3f& vectorB)
  {
    vector3f result;
    result.x = vectorA.x - vectorB.x;
    result.y = vectorA.y - vectorB.y;
    result.z = vectorA.z - vectorB.z;
    return result;
  }


int main(int argc, char** argv)
{

  // VehicleWrapper vehicle_wrapper(1134899, "5a29a02e49f8b42c83a3ccde7c79a1b1cf82e54bca131ba137471964aa0c906f", "/dev/ttyACM0", "/dev/ttyUSB0", 1000000, false);
  ros::init(argc, argv, "flight_control_node");
  ros::NodeHandle nh;
  task_control_client = nh.serviceClient<FlightTaskControl>("/flight_task_control");
  auto set_go_home_altitude_client = nh.serviceClient<SetGoHomeAltitude>("/set_go_home_altitude");
  auto get_go_home_altitude_client = nh.serviceClient<GetGoHomeAltitude>("get_go_home_altitude");
  auto set_current_point_as_home_client = nh.serviceClient<SetCurrentAircraftLocAsHomePoint>("/set_current_aircraft_point_as_home");
  auto enable_horizon_avoid_client  = nh.serviceClient<SetAvoidEnable>("/set_horizon_avoid_enable");
  auto enable_upward_avoid_client   = nh.serviceClient<SetAvoidEnable>("/set_upwards_avoid_enable");
  auto get_avoid_enable_client      = nh.serviceClient<GetAvoidEnable>("get_avoid_enable_status");
  auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>("obtain_release_control_authority");
  auto emergency_brake_client       = nh.serviceClient<dji_osdk_ros::EmergencyBrake>("emergency_brake");

  attitudeSub      = nh.subscribe("dji_osdk_ros/attitude", 10, &attitudeSubCallback);
  rtkPositionSub   = nh.subscribe("dji_osdk_ros/rtk_position", 10, &rtkPositionSubCallback);
  gpsPositionSub   = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
  heightSub        = nh.subscribe("dji_osdk_ros/height_above_takeoff", 10, &heightSubCallback);


  set_joystick_mode_client = nh.serviceClient<SetJoystickMode>("set_joystick_mode");
  joystick_action_client   = nh.serviceClient<JoystickAction>("joystick_action");

  ros::Duration(1).sleep();
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Monitored Takeoff + Landing                                |"
      << std::endl;
  std::cout
      << "| [b] Monitored Takeoff + Position Control + Landing             |"
      << std::endl;
  std::cout << "| [c] Monitored Takeoff + Position Control + Force Landing "
               "Avoid Ground  |"
            << std::endl;
  std::cout << "| [d] Monitored Takeoff + Velocity Control + Landing |"
            << std::endl;
  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;
  EmergencyBrake emergency_brake;
  FlightTaskControl control_task;
  ObtainControlAuthority obtainCtrlAuthority;
  
  obtainCtrlAuthority.request.enable_obtain = true;
  obtain_ctrl_authority_client.call(obtainCtrlAuthority);

  float32_t homeRTKLat = (rtk_position_.latitude)*DEG2RAD;
  float32_t homeRTKLong = (rtk_position_.longitude)*DEG2RAD;

               
  switch (inputChar)
  {
    case 'a':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("Land request sending ...");
          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
        ROS_ERROR_STREAM("Takeoff task failed");
        break;
      }
    case 'b':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();
          

          ROS_INFO_STREAM("Move by position offset request sending ...");
          moveByPosOffset(control_task, {0, 0.0, 0.8, 0.0}, 0.1, 1, 0.3);
          ROS_INFO_STREAM("Step 1 over!");
          float32_t homeRTKHeight = height_above_takeoff_.data;
          ros::Duration(2.0).sleep();
          moveByPosOffset(control_task, {-20, 0.0, 3.0, 0.0}, 0.5, 1, 1);
          ROS_INFO_STREAM("Step 2 over!");
          ros::Duration(5.0).sleep();
          returnToHome(control_task, {20, 0.0, -3.0, 0.0}, 1.0, 1, 1, homeRTKLat, homeRTKLong, homeRTKHeight);
          ROS_INFO_STREAM("Step 3 over!");
          ros::Duration(1.0).sleep();


          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          ROS_INFO_STREAM("Landing request sending ...");
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
        break;
      }
    case 'c':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if (control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2.0).sleep();

          ROS_INFO_STREAM("turn on Horizon_Collision-Avoidance-Enabled");
          SetAvoidEnable horizon_avoid_req;
          horizon_avoid_req.request.enable = true;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("turn on Upwards-Collision-Avoidance-Enabled");
          SetAvoidEnable upward_avoid_req;
          upward_avoid_req.request.enable = true;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          GetAvoidEnable getAvoidEnable;
          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          ROS_INFO_STREAM("Move by position offset request sending ...");
          ROS_INFO_STREAM("Move to higher altitude");
          moveByPosOffset(control_task, {0.0, 0.0, 30.0, 0.0}, 0.8, 1, 1);
          ROS_INFO_STREAM("Move a short distance");
          moveByPosOffset(control_task, {10.0, 0.0, 0.0, 0.0}, 0.8, 1, 1);

          ROS_INFO_STREAM("Set aircraft current position as new home location");
          SetCurrentAircraftLocAsHomePoint home_set_req;
          set_current_point_as_home_client.call(home_set_req);
          if(home_set_req.response.result == false)
          {
            ROS_ERROR_STREAM("Set current position as Home, FAILED");
            break;
          }


          ROS_INFO_STREAM("Get current go home altitude");
          GetGoHomeAltitude current_go_home_altitude;
          get_go_home_altitude_client.call(current_go_home_altitude);
          if(current_go_home_altitude.response.result == false)
          {
            ROS_ERROR_STREAM("Get altitude for go home FAILED");
            break;
          }
          ROS_INFO("Current go home altitude is :%d m", current_go_home_altitude.response.altitude);

          ROS_INFO_STREAM("Set new go home altitude");
          SetGoHomeAltitude altitude_go_home;
          altitude_go_home.request.altitude = 50;
          set_go_home_altitude_client.call(altitude_go_home);
          if(altitude_go_home.response.result == false)
          {
            ROS_ERROR_STREAM("Set altitude for go home FAILED");
            break;
          }

          get_go_home_altitude_client.call(current_go_home_altitude);
          if(current_go_home_altitude.response.result == false)
          {
            ROS_ERROR_STREAM("Get altitude for go home FAILED");
            break;
          }
          ROS_INFO("Current go home altitude is :%d m", current_go_home_altitude.response.altitude);

          ROS_INFO_STREAM("Move to another position");
          moveByPosOffset(control_task, {50.0, 0.0, 0.0, 0.0} , 0.8, 1, 1);

          ROS_INFO_STREAM("Shut down Horizon_Collision-Avoidance-Enabled");
          horizon_avoid_req.request.enable = false;
          enable_horizon_avoid_client.call(horizon_avoid_req);
          if(horizon_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Disable Horizon Avoid FAILED");
          }

          ROS_INFO_STREAM("Shut down Upwards-Collision-Avoidance-Enabled");
          upward_avoid_req.request.enable = false;
          enable_upward_avoid_client.call(upward_avoid_req);
          if(upward_avoid_req.response.result == false)
          {
            ROS_ERROR_STREAM("Enable Upward Avoid FAILED");
          }

          get_avoid_enable_client.call(getAvoidEnable);
          if (getAvoidEnable.response.result)
          {
            ROS_INFO("get horizon avoid enable status:%d, get upwards avoid enable status:%d",
                     getAvoidEnable.response.horizon_avoid_enable_status,
                     getAvoidEnable.response.upwards_avoid_enable_status);
          }

          ROS_INFO_STREAM("Go home...");

          control_task.request.task = FlightTaskControl::Request::TASK_GOHOME_AND_CONFIRM_LANDING;
          task_control_client.call(control_task);
          if(control_task.response.result == false)
          {
            ROS_ERROR_STREAM("GO HOME FAILED");
          }
          break;
        }
      }
    case 'd':
      {
        control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
        ROS_INFO_STREAM("Takeoff request sending ...");
        task_control_client.call(control_task);
        if(control_task.response.result == false)
        {
          ROS_ERROR_STREAM("Takeoff task failed");
          break;
        }

        if(control_task.response.result == true)
        {
          ROS_INFO_STREAM("Takeoff task successful");
          ros::Duration(2).sleep();

          velocityAndYawRateCtrl( {0, 0, 0.4, 0}, 2000);
          ROS_INFO_STREAM("Step 1 over!EmergencyBrake for 2s\n");
          emergency_brake_client.call(emergency_brake);
          ros::Duration(2).sleep();
          velocityAndYawRateCtrl({-2, 0, 0.333, 0}, 10000);
          ROS_INFO_STREAM("Step 2 over!EmergencyBrake for 2s\n");
          emergency_brake_client.call(emergency_brake);
          ros::Duration(2).sleep();
          velocityAndYawRateCtrl({2, 0, -0.333, 0}, 10000);
          ROS_INFO_STREAM("Step 3 over!EmergencyBrake for 2s\n");
          emergency_brake_client.call(emergency_brake);
          ros::Duration(2).sleep();

          control_task.request.task = FlightTaskControl::Request::TASK_LAND;
          ROS_INFO_STREAM("Landing request sending ...");
          task_control_client.call(control_task);
          if(control_task.response.result == true)
          {
            ROS_INFO_STREAM("Land task successful");
            break;
          }
          ROS_INFO_STREAM("Land task failed.");
          break;
        }
        break;
      }
    default:
      break;
  }

  ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");

  ros::spin();
  return 0;
}


// bool moveByPosOffset(FlightTaskControl& task, const JoystickCommand &offsetDesired,
//                     float posThresholdInM,
//                     float yawThresholdInDeg)
// {
//   task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
//   task.request.joystickCommand.x = offsetDesired.x;
//   task.request.joystickCommand.y = offsetDesired.y;
//   task.request.joystickCommand.z = offsetDesired.z;
//   task.request.joystickCommand.yaw = offsetDesired.yaw;
//   task.request.posThresholdInM   = posThresholdInM;
//   task.request.yawThresholdInDeg = yawThresholdInDeg;

//   task_control_client.call(task);
//   return task.response.result;
// }


bool moveByPosOffset(FlightTaskControl& task, const JoystickCommand &offsetDesired,
                    float posThresholdInM,
                    float yawThresholdInDeg, float speedFactor)
{
  int responseTimeout = 1;
  int timeoutInMilSec = 40000;
  int controlFreqInHz = 50;  // Hz
  int cycleTimeInMs = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
  int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
  int elapsedTimeInMs = 0;
  int withinBoundsCounter = 0;
  int outOfBounds = 0;
  int brakeCounter = 0;
  int counter = 0;
  SetJoystickMode joystickMode;
  JoystickAction joystickAction;

  // get origin position and relative height(from home point)of aircraft.
  float32_t currentRTKLat = (rtk_position_.latitude)*DEG2RAD;
  float32_t currentRTKLong = (rtk_position_.longitude)*DEG2RAD;
  float32_t currentHeight = height_above_takeoff_.data;

  float32_t originalRTKLat = currentRTKLat;
  float32_t originalRTKLong = currentRTKLong;
  float32_t originalHeight = currentHeight;

  vector3f localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, originalHeight);
  vector3f offsetRemaining = vector3FSub(offsetDesired, localoffset);
  
  double yawDesiredInDeg = offsetDesired.yaw;
  double yawDesiredRad     = DEG2RAD * yawDesiredInDeg;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;
  quaternion currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
  double currentYawInRad = quaternionToEulerAngle(currentQuaternion).z;
  double yawRemainingInRad = yawDesiredRad - currentYawInRad;

  // Compute distance between the desired and original point
  float distance = sqrt(pow(offsetDesired.x, 2) + pow(offsetDesired.y, 2) + pow(offsetDesired.z, 2));

  // Compute time it takes to move to desired offset at desired speed
  float timeInMs = (distance / speedFactor) * 1000;

  // Compute number of cycles needed
  float numCycle = timeInMs / cycleTimeInMs;

  // Compute offset per cycle
  float offsetPerCycleX = offsetDesired.x / numCycle;
  float offsetPerCycleY = offsetDesired.y / numCycle;
  float offsetPerCycleZ = offsetDesired.z / numCycle;

  //Compute speed per axis
  float cmdX = offsetDesired.x / (timeInMs / 1000);
  float cmdY = offsetDesired.y / (timeInMs / 1000);
  float cmdZ = offsetDesired.z / (timeInMs / 1000);

  double originTime  = ros::Time::now().toSec();
  double currentTime = originTime;

  while(elapsedTimeInMs <= timeInMs)
  {
    joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
    joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
    joystickMode.request.yaw_mode = joystickMode.request.YAW_ANGLE;
    joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
    joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
    set_joystick_mode_client.call(joystickMode);

    joystickAction.request.joystickCommand.x = cmdX;
    joystickAction.request.joystickCommand.y = cmdY;
    joystickAction.request.joystickCommand.z = cmdZ;
    joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;

    currentTime = ros::Time::now().toSec();
    elapsedTimeInMs = (currentTime - originTime) * 1000;
    joystick_action_client.call(joystickAction);

    ROS_INFO("elapsedTime: %d", elapsedTimeInMs);

    currentRTKLat = (rtk_position_.latitude)*DEG2RAD;
    currentRTKLong = (rtk_position_.longitude)*DEG2RAD;
    currentHeight = height_above_takeoff_.data;
    ROS_INFO("current position: %f, %f, %f", currentRTKLat, currentRTKLong, currentHeight);

    currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
    currentYawInRad = quaternionToEulerAngle(currentQuaternion).z;

    localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, originalHeight);
    offsetRemaining = vector3FSub(offsetDesired, localoffset);
    yawRemainingInRad = yawDesiredRad - currentYawInRad;
    ROS_INFO("offset remaining: %f, %f, %f, %f", offsetRemaining.x, offsetRemaining.y, offsetRemaining.z, vectorNorm(offsetRemaining));
    usleep(cycleTimeInMs * 1000);
  }
  return true;
}

void returnToHome(FlightTaskControl& task, const JoystickCommand &offsetDesired,
                     float posThresholdInM, float yawThresholdInDeg, float speedFactor, float32_t homeRTKLat, float32_t homeRTKLong, float32_t homeRTKHeight)
{
  int responseTimeout = 1;
  int timeoutInMilSec = 40000;
  int controlFreqInHz = 50;  // Hz
  int cycleTimeInMs = 1000 / controlFreqInHz;
  int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
  int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
  int elapsedTimeInMs = 0;
  int withinBoundsCounter = 0;
  int outOfBounds = 0;
  int brakeCounter = 0;
  SetJoystickMode joystickMode;
  JoystickAction joystickAction;

  // get origin position and relative height(from home point)of aircraft.
  float32_t currentRTKLat = (rtk_position_.latitude)*DEG2RAD;
  float32_t currentRTKLong = (rtk_position_.longitude)*DEG2RAD;
  float32_t currentHeight = height_above_takeoff_.data;

  vector3f offsetRemaining = offsetRemainingFromRTKOffset(homeRTKLat, homeRTKLong, currentRTKLat, currentRTKLong, 2.0, currentHeight);
  
  double yawDesiredInDeg = offsetDesired.yaw;
  double yawDesiredRad     = DEG2RAD * yawDesiredInDeg;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;
  quaternion currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
  double currentYawInRad = quaternionToEulerAngle(currentQuaternion).z;
  double yawRemainingInRad = yawDesiredRad - currentYawInRad;

  // Compute distance between the desired and original point
  float distance = sqrt(pow(offsetRemaining.x, 2) + pow(offsetRemaining.y, 2) + pow(offsetRemaining.z, 2));

  // Compute time it takes to move to desired offset at desired speed
  float timeInMs = (distance / speedFactor) * 1000;

  // Compute number of cycles needed
  float numCycle = timeInMs / cycleTimeInMs;

  // Compute offset per cycle
  float offsetPerCycleX = offsetRemaining.x / numCycle;
  float offsetPerCycleY = offsetRemaining.y / numCycle;
  float offsetPerCycleZ = offsetDesired.z / numCycle;

  //Compute speed per axis
  float cmdX = offsetRemaining.x / (timeInMs / 1000);
  float cmdY = offsetRemaining.y / (timeInMs / 1000);
  float cmdZ = offsetDesired.z / (timeInMs / 1000);

  double originTime  = ros::Time::now().toSec();
  double currentTime = originTime;

  while(elapsedTimeInMs <= timeInMs)
  {
    joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
    joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
    joystickMode.request.yaw_mode = joystickMode.request.YAW_ANGLE;
    joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_GROUND;
    joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
    set_joystick_mode_client.call(joystickMode);

    joystickAction.request.joystickCommand.x = cmdX;
    joystickAction.request.joystickCommand.y = cmdY;
    joystickAction.request.joystickCommand.z = cmdZ;
    joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;

    currentTime = ros::Time::now().toSec();
    elapsedTimeInMs = (currentTime - originTime) * 1000;
    joystick_action_client.call(joystickAction);

    ROS_INFO("elapsedTime: %d", elapsedTimeInMs);

    currentRTKLat = (rtk_position_.latitude)*DEG2RAD;
    currentRTKLong = (rtk_position_.longitude)*DEG2RAD;
    currentHeight = height_above_takeoff_.data;
    ROS_INFO("current position: %f, %f, %f", currentRTKLat, currentRTKLong, currentHeight);

    currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
    currentYawInRad = quaternionToEulerAngle(currentQuaternion).z;

    vector3f offsetRemaining = offsetRemainingFromRTKOffset(homeRTKLat, homeRTKLong, currentRTKLat, currentRTKLong, 2.0, currentHeight);
    yawRemainingInRad = yawDesiredRad - currentYawInRad;
    ROS_INFO("offset remaining: %f, %f, %f, %f", offsetRemaining.x, offsetRemaining.y, offsetRemaining.z, vectorNorm(offsetRemaining));
    
    if (vectorNorm(offsetRemaining) < posThresholdInM) {
      ROS_INFO("Called position control function");
      ROS_INFO("offset remaining before: %f, %f, %f, %f", offsetRemaining.x, offsetRemaining.y, offsetRemaining.z, vectorNorm(offsetRemaining));
      
      task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
      task.request.joystickCommand.x = offsetRemaining.x;
      task.request.joystickCommand.y = offsetRemaining.y;
      task.request.joystickCommand.z = offsetRemaining.z;
      task.request.joystickCommand.yaw = offsetDesired.yaw;
      task.request.posThresholdInM   = 0.2;
      task.request.yawThresholdInDeg = 1;
      task_control_client.call(task);    
      
      // ros::Duration(3).sleep();
      currentRTKLat = (rtk_position_.latitude)*DEG2RAD;
      currentRTKLong = (rtk_position_.longitude)*DEG2RAD;
      currentHeight = height_above_takeoff_.data;

      currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
      currentYawInRad = quaternionToEulerAngle(currentQuaternion).z;
      vector3f offsetRemaining = localOffsetFromRTKOffset(homeRTKLat, homeRTKLong, currentRTKLat, currentRTKLong, 2.0, currentHeight);
      yawRemainingInRad = yawDesiredRad - currentYawInRad;
      ROS_INFO("offset remaining after: %f, %f, %f, %f", offsetRemaining.x, offsetRemaining.y, offsetRemaining.z, vectorNorm(offsetRemaining));
      break;
    }
    usleep(cycleTimeInMs * 1000);
  }
}



// bool moveByPosOffset(FlightTaskControl& task, const JoystickCommand &offsetDesired,
//                     float posThresholdInM, float yawThresholdInDeg)
// {
//   // vector3f offsetDesiredxyz = {offsetDesired.x, offsetDesired.y, offsetDesired.z};
//   float yawDesiredInDeg = offsetDesired.yaw;

//   task.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
  
//   int responseTimeout = 1;
//   int timeoutInMilSec = 40000;
//   int controlFreqInHz = 50;  // Hz
//   int cycleTimeInMs = 1000 / controlFreqInHz;
//   int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
//   int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
//   int elapsedTimeInMs = 0;
//   int withinBoundsCounter = 0;
//   int outOfBounds = 0;
//   int brakeCounter = 0;
//   int speedFactor = 10;
//   float zSpeedFactor = 0.5;

//   //! get origin position and relative height(from home point)of aircraft.
//   float32_t currentRTKLat = (gps_position_.latitude)*DEG2RAD;
//   float32_t currentRTKLong = (gps_position_.longitude)*DEG2RAD;
//   float32_t currentHeight = height_above_takeoff_.data;

//   float32_t originalRTKLat = currentRTKLat;
//   float32_t originalRTKLong = currentRTKLong;
//   float32_t originalHeight = currentHeight;

//   vector3f localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, originalHeight);
//   vector3f offsetRemaining = vector3FSub(offsetDesired, localoffset);

//   double yawDesiredRad     = DEG2RAD * yawDesiredInDeg;
//   double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

//   quaternion currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
//   double yawInRad = quaternionToEulerAngle(currentQuaternion).z;

//   vector3f positionCommand;

//   if (offsetDesired.x > 0)
//     positionCommand.x = (offsetDesired.x < speedFactor) ? offsetDesired.x : speedFactor;
//   else if (positionCommand.x < 0)
//     positionCommand.x = (offsetDesired.x > -1 * speedFactor) ? offsetDesired.x : -1 * speedFactor;
//   else
//     positionCommand.x = 0;

//   if (offsetDesired.y > 0)
//     positionCommand.y = (offsetDesired.y < speedFactor) ? offsetDesired.y : speedFactor;
//   else if (positionCommand.y < 0)
//     positionCommand.y = (offsetDesired.y > -1 * speedFactor) ? offsetDesired.y : -1 * speedFactor;
//   else
//     positionCommand.y = 0;

//   if (offsetDesired.z > 0)
//     positionCommand.z = (offsetDesired.z < zSpeedFactor) ? offsetDesired.z : zSpeedFactor;
//   else if (positionCommand.z < 0)
//     positionCommand.z = (offsetDesired.z > -1 * zSpeedFactor) ? offsetDesired.z : -1 * zSpeedFactor;
//   else
//     positionCommand.z = 0;


//   // Compute distance between the desired and original point
//   float distance = sqrt(pow(offsetDesired.x, 2) + pow(offsetDesired.y, 2) + pow(offsetDesired.z, 2));

//   // Compute time it takes to move to desired offset at desired speed
//   float timeInMs = (distance / speedFactor) * 1000;

//   // Compute number of cycles needed
//   float numCycle = timeInMs / cycleTimeInMs;

//   // Compute offset per cycle
//   float offsetPerCycleX = offsetDesired.x / numCycle;
//   float offsetPerCycleY = offsetDesired.y / numCycle;
//   float offsetPerCycleZ = offsetDesired.z / numCycle;

//   int originTime  = ros::Time::now().toSec();
//   int currentTime = originTime;
//   int elapsedTime = currentTime - originTime;

//   while (elapsedTimeInMs < timeoutInMilSec) {
//     task.request.joystickCommand.x = offsetPerCycleX;
//     task.request.joystickCommand.y = offsetPerCycleY;
//     task.request.joystickCommand.z = offsetPerCycleZ;
//     task.request.joystickCommand.yaw = yawDesiredInDeg;
//     task.request.posThresholdInM   = posThresholdInM;
//     task.request.yawThresholdInDeg = yawThresholdInDeg;

//     task_control_client.call(task);

//     // usleep(cycleTimeInMs * 1000);
//     elapsedTimeInMs += cycleTimeInMs;
//     currentTime = ros::Time::now().toSec();
//     elapsedTime = currentTime - originTime;
//     ROS_INFO("elapsedTime: %d", elapsedTime);

//     currentRTKLat = (gps_position_.latitude)*DEG2RAD;
//     currentRTKLong = (gps_position_.longitude)*DEG2RAD;
//     currentHeight = height_above_takeoff_.data;
//     ROS_INFO("current position: %f, %f, %f", currentRTKLat, currentRTKLong, currentHeight);

//     currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
//     yawInRad = quaternionToEulerAngle(currentQuaternion).z;

//     localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, originalHeight);
//     offsetRemaining = vector3FSub(offsetDesired, localoffset);
//     ROS_INFO("offset desired: %f, %f, %f", offsetDesired.x, offsetDesired.y, offsetDesired.z);
//     ROS_INFO("offset local: %f, %f, %f", localoffset.x, localoffset.y, localoffset.z);
//     ROS_INFO("offset remaining: %f, %f, %f", offsetRemaining.x, offsetRemaining.y, offsetRemaining.z);

//     positionCommand = offsetRemaining;
//     // horizCommandLimit(speedFactor, positionCommand.x, positionCommand.y);
//     // vertiCommandLimit(zSpeedFactor, positionCommand.z);
//     // ROS_INFO("position command: %f, %f, %f", positionCommand.x, positionCommand.y, positionCommand.z);


//     if (vectorNorm(offsetRemaining) < posThresholdInM &&
//         std::fabs(yawInRad / DEG2RAD - yawDesiredInDeg) < yawThresholdInDeg) {
//       //! 1. We are within bounds; start incrementing our in-bound counter
//       withinBoundsCounter += cycleTimeInMs;
//     } else {
//       if (withinBoundsCounter != 0) {
//         //! 2. Start incrementing an out-of-bounds counter
//         outOfBounds += cycleTimeInMs;
//       }
//     }
//     //! 3. Reset withinBoundsCounter if necessary
//     if (outOfBounds > outOfControlBoundsTimeLimit) {
//       withinBoundsCounter = 0;
//       outOfBounds = 0;
//     }
//     //! 4. If within bounds, set flag and break
//     if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
//       break;
//     }
//   }

//   while (brakeCounter < withinControlBoundsTimeReqmt) {
//     //! TODO: remove emergencyBrake
//     // emergency_brake_client.call(emergency_brake);
//     usleep(cycleTimeInMs * 1000);
//     brakeCounter += cycleTimeInMs;
//   }

//   if (elapsedTimeInMs >= timeoutInMilSec) {
//     std::cout << "Task timeout!\n";
//     return false;
//   }
  
//   return task.response.result;
// }



void velocityAndYawRateCtrl(const JoystickCommand &offsetDesired, uint32_t timeMs)
{
  double originTime  = 0;
  double currentTime = 0;
  uint64_t elapsedTimeInMs = 0;
  
  SetJoystickMode joystickMode;
  JoystickAction joystickAction;

  joystickMode.request.horizontal_mode = joystickMode.request.HORIZONTAL_VELOCITY;
  joystickMode.request.vertical_mode = joystickMode.request.VERTICAL_VELOCITY;
  joystickMode.request.yaw_mode = joystickMode.request.YAW_RATE;
  joystickMode.request.horizontal_coordinate = joystickMode.request.HORIZONTAL_BODY;
  joystickMode.request.stable_mode = joystickMode.request.STABLE_ENABLE;
  set_joystick_mode_client.call(joystickMode);

  joystickAction.request.joystickCommand.x = offsetDesired.x;
  joystickAction.request.joystickCommand.y = offsetDesired.y;
  joystickAction.request.joystickCommand.z = offsetDesired.z;
  joystickAction.request.joystickCommand.yaw = offsetDesired.yaw;

  originTime  = ros::Time::now().toSec();
  currentTime = originTime;
  elapsedTimeInMs = (currentTime - originTime)*1000;

  while(elapsedTimeInMs <= timeMs)
  {
    currentTime = ros::Time::now().toSec();
    elapsedTimeInMs = (currentTime - originTime) * 1000;
    joystick_action_client.call(joystickAction);
  }
}