/** @file main.cpp
 *  @version 3.7
 *  @date July, 2018
 *
 *  @brief
 *  DJISDKNode
 *
 *  @copyright 2018 DJI. All rights reserved.
 *
 */
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_sdk/dji_sdk_node.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

struct quaternion {double q0, q1, q2, q3;};
struct EulerAngles {double x, y, z;};
struct vector3f {float x, y, z;};

geometry_msgs::QuaternionStamped attitude_data_;
sensor_msgs::NavSatFix rtk_position_;
sensor_msgs::NavSatFix gps_position_;
std_msgs::Float32 height_above_takeoff_;

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;

ros::Subscriber rtkPositionSub;
ros::Subscriber gpsPositionSub;
ros::Subscriber attitudeSub;
ros::Subscriber heightSub;
ros::Subscriber gpsSub;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

bool moveByPosOffset(DJISDKNode* dji_sdk_node_, float xSP, float ySP, float zSP, float yawSP,
                    float posThresholdInM, float yawThresholdInDeg, float sec, float zThresholdInM, float32_t homeHeight);

bool obtain_control();

bool monitoredTakeoff();

bool set_local_position();

void attitudeSubCallback(const geometry_msgs::QuaternionStampedConstPtr& attitudeData)
{attitude_data_ = *attitudeData;}

void rtkPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& rtkPosition)
{rtk_position_ = *rtkPosition;}

void gpsPositionSubCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsPosition)
{gps_position_ = *gpsPosition;}

void heightSubCallback(const std_msgs::Float32::ConstPtr& heightAboveTakeoff)
{height_above_takeoff_ = *heightAboveTakeoff;}

vector3f localOffsetFromRTKOffset(
  const double& targetLat, const double& targetLong, const double& originLat,
  const double& originLong, const double& targetHeight, const double& originHeight)
{
  vector3f deltaNed;
  double deltaLon = targetLong - originLong;
  double deltaLat = targetLat -  originLat;
  deltaNed.x = deltaLat * C_EARTH;
  deltaNed.y = deltaLon * C_EARTH * cos(targetLat);
  deltaNed.z = targetHeight - originHeight;
  return deltaNed;
}

vector3f vector3FSub(const vector3f& vectorA, const vector3f& vectorB)
{
  vector3f result;
  result.x = vectorA.x - vectorB.x;
  result.y = vectorA.y - vectorB.y;
  result.z = vectorA.z - vectorB.z;
  return result;
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

float32_t vectorNorm(vector3f v)
{
  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dji_sdk");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  DJISDKNode* dji_sdk_node = new DJISDKNode(nh, nh_private, argc, argv);

  attitudeSub      = nh.subscribe("dji_osdk_ros/attitude", 10, &attitudeSubCallback);
  rtkPositionSub   = nh.subscribe("dji_osdk_ros/rtk_position", 10, &rtkPositionSubCallback);
  gpsPositionSub   = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
  heightSub        = nh.subscribe("dji_osdk_ros/height_above_takeoff", 10, &heightSubCallback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_ENUposition_yaw", 10);
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_osdk_ros/flight_control_setpoint_generic", 10);

  ros::Duration(1).sleep();
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  ros::Duration(10).sleep();
  sdk_ctrl_authority_service = nh.serviceClient<dji_osdk_ros::SDKControlAuthority>("dji_osdk_ros/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_osdk_ros::DroneTaskControl>("dji_osdk_ros/drone_task_control");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  bool land_result; 

  double homeHeight = rtk_position_.altitude;

  // Take off   
  dji_osdk_ros::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = dji_osdk_ros::DroneTaskControl::Request::TASK_TAKEOFF;
  drone_task_service.call(droneTaskControl);
  takeoff_result = droneTaskControl.response.result;
  if (takeoff_result){
    ROS_INFO("Drone taking off!");
  }
  ros::Duration(7).sleep();

  // Move drone by x, y, z, yaw command
  moveByPosOffset(dji_sdk_node, 0, 0, 2, 0.0, 0.5, 1, 5, 0.5, homeHeight);
  ROS_INFO_STREAM("Step 1 over!");  
  ros::Duration(2).sleep();
  moveByPosOffset(dji_sdk_node, -20, 0, 5, 0.0, 0.5, 1, 25, 0.5, homeHeight);
  ROS_INFO_STREAM("Step 2 over!");
  ros::Duration(2).sleep();
  moveByPosOffset(dji_sdk_node, 20, 0, 2, 0.0, 0.5, 1, 25, 0.5, homeHeight);
  ROS_INFO_STREAM("Step 3 over!");

  // Land
  droneTaskControl.request.task = dji_osdk_ros::DroneTaskControl::Request::TASK_LAND;
  drone_task_service.call(droneTaskControl);
  land_result = droneTaskControl.response.result;
  if (land_result){
    ROS_INFO("Drone landing!");
  }

  ros::waitForShutdown();

  delete dji_sdk_node;
  dji_sdk_node = NULL;

  return 0;
}


bool obtain_control()
{
  dji_osdk_ros::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}


bool moveByPosOffset(DJISDKNode* dji_sdk_node_, 
                    float xSP, float ySP, float zSP, float yawSP,
                    float posThresholdInM, float yawThresholdInDeg, float sec, float zThresholdInM, float32_t homeHeight)
{
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_POSITION |
                       DJI::OSDK::Control::HORIZONTAL_POSITION |
                       DJI::OSDK::Control::YAW_ANGLE |
                       DJI::OSDK::Control::HORIZONTAL_GROUND |
                       DJI::OSDK::Control::STABLE_ENABLE);
  
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
  int speedFactor = 1;

  vector3f offsetDesired = {xSP, ySP, zSP};
  float yawDesiredInDeg = yawSP;
  double yawDesiredRad     = deg2rad * yawDesiredInDeg;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  vector3f positionCommand;

  // Get original position and current position from GPS (RTK) data
  double currentRTKLat = (rtk_position_.latitude)*deg2rad;
  double currentRTKLong = (rtk_position_.longitude)*deg2rad;
  double currentHeight = rtk_position_.altitude;
  double originalRTKLat = currentRTKLat;
  double originalRTKLong = currentRTKLong;
  double originalHeight = currentHeight;

  // Get initial offset. We will update this in a loop later
  vector3f localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, originalHeight);
  vector3f offsetRemaining = vector3FSub(offsetDesired, localoffset);
  quaternion currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
  double yawInRad = quaternionToEulerAngle(currentQuaternion).z;

  if (offsetDesired.x > 0)
    positionCommand.x = (offsetDesired.x < speedFactor) ? offsetDesired.x : speedFactor;
  else if (offsetDesired.x < 0)
    positionCommand.x = (offsetDesired.x > -1 * speedFactor) ? offsetDesired.x : -1 * speedFactor;
  else
    positionCommand.x = 0;

  if (offsetDesired.y > 0)
    positionCommand.y = (offsetDesired.y < speedFactor) ? offsetDesired.y : speedFactor;
  else if (offsetDesired.y < 0)
    positionCommand.y = (offsetDesired.y > -1 * speedFactor) ? offsetDesired.y : -1 * speedFactor;
  else
    positionCommand.y = 0;

  positionCommand.z = originalHeight-homeHeight;


  float deltaZ =  offsetDesired.z - (originalHeight-homeHeight);
  float offsetZ = offsetDesired.z;
  float distance = sqrt(pow(offsetDesired.x, 2) + pow(offsetDesired.y, 2) + pow(offsetZ, 2));
  float timeInMs = sec * 1000;

  // Compute number of cycles needed for z axis
  float numCycle = timeInMs / cycleTimeInMs;
  float offsetPerCycleZ = offsetZ / numCycle;

  while (elapsedTimeInMs < timeoutInMilSec) {
    // Send the command to the aircraft
    dji_sdk_node_->flightControl(ctrl_flag, positionCommand.x, positionCommand.y, positionCommand.z, yawDesiredInDeg);
    usleep(cycleTimeInMs * 1000);

    elapsedTimeInMs += cycleTimeInMs;
    ROS_INFO("elapsedTimeInMs: %d", elapsedTimeInMs);
    
    // Update current position
    currentRTKLat = (rtk_position_.latitude)*deg2rad;
    currentRTKLong = (rtk_position_.longitude)*deg2rad;
    currentHeight = rtk_position_.altitude;
    ROS_INFO("current position: %f, %f, %f", currentRTKLat, currentRTKLong, currentHeight-homeHeight);
    currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
    yawInRad = quaternionToEulerAngle(currentQuaternion).z;

    // See how much farther we have to go
    localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, homeHeight);
    offsetRemaining = vector3FSub(offsetDesired, localoffset);
    ROS_INFO("offset remaining: %f, %f, %f, %f", offsetRemaining.x, offsetRemaining.y, offsetRemaining.z, vectorNorm(offsetRemaining));

    // See if we need to modify the setpoint
    if (fabs(offsetRemaining.x) < speedFactor) {
      positionCommand.x = offsetRemaining.x;
    }
    if (fabs(offsetRemaining.y) < speedFactor) {
      positionCommand.y = offsetRemaining.y;
    }

    if (fabs(offsetRemaining.z) > zThresholdInM) {
      if (deltaZ > 0){
        positionCommand.z += offsetPerCycleZ;
      }
      if (deltaZ < 0){
        positionCommand.z -= offsetPerCycleZ;
      }
      
    }

    if (vectorNorm(offsetRemaining) < posThresholdInM) {
      //! 1. We are within bounds; start incrementing our in-bound counter
      withinBoundsCounter += cycleTimeInMs;
    } else {
      if (withinBoundsCounter != 0) {
        //! 2. Start incrementing an out-of-bounds counter
        outOfBounds += cycleTimeInMs;
      }
    }
    //! 3. Reset withinBoundsCounter if necessary
    if (outOfBounds > outOfControlBoundsTimeLimit) {
      withinBoundsCounter = 0;
      outOfBounds = 0;
    }
    //! 4. If within bounds, set flag and break
    if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
      break;
    }
  }

  while (brakeCounter < withinControlBoundsTimeReqmt) {
    //! TODO: remove emergencyBrake
    // emergency_brake_client.call(emergency_brake);
    usleep(cycleTimeInMs * 1000);
    brakeCounter += cycleTimeInMs;
  }

  if (elapsedTimeInMs >= timeoutInMilSec) {
    std::cout << "Task timeout!\n";
    return false;
  }
  
  return true;
}