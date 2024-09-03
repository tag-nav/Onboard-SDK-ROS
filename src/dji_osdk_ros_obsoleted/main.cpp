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
struct vector3f {double x, y, z;};

struct DataRow {
    double time;
    double x;
    double y;
    double z;
    double x_input;
    double y_input;
    double z_input;
    double x_error;
    double y_error;
    double z_error;
    double cur_minus_gnd_height;
    double ground_height;
};

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
                    float posThresholdInM, float yawThresholdInDeg, float zThresholdInM, float32_t groundHeight, string filename);

bool generateSinusoidalTraj(DJISDKNode* dji_sdk_node_, float amplitude, float freq, float duration, float32_t originalHeight, string filename);

bool generateSingleCmd(DJISDKNode* dji_sdk_node_, float xSP, float ySP, float zSP, float yawSP, string filename);

bool PIDTracking(DJISDKNode* dji_sdk_node_, 
                    float xSP, float ySP, float zSP, float yawSP, float speed, float32_t groundHeight, string filename);

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

// Function to write the header to a CSV file
void writeCSVHeader(const std::string& filename) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filename << std::endl;
        return;
    }

    // Write the header
    file << "time,x,y,z,x_input,y_input,z_input, x_error, y_error, z_error, cur_minus_gnd_height, ground_height\n";

    file.close();

    if (file.fail()) {
        std::cerr << "Error: Could not write to the file " << filename << std::endl;
    }
}

void writeCSVRow(const std::string& filename, const DataRow& row) {
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << filename << std::endl;
        return;
    }

    // Write the data row
    file << fixed << setprecision(9) << row.time << "," << row.x << "," << row.y << "," << row.z 
    << "," << row.x_input << "," << row.y_input << "," << row.z_input <<","
    <<row.x_error<<","<<row.y_error<<","<<row.z_error<<"," 
    <<row.cur_minus_gnd_height<<","
    <<row.ground_height<<"\n";

    file.close();

    if (file.fail()) {
        std::cerr << "Error: Could not write to the file " << filename << std::endl;
    }
}

class PID {
  public:
    PID(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

  double calculate(double setpoint, double measured_value, double dt) {
    double error = setpoint - measured_value;
    // ROS_INFO("error: %f", error);
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    double output = kp_ * error + ki_* integral_ + kd_ * derivative;
    // ROS_INFO("kp, ki, kd: %f, %f, %f", kp_*error, ki_*integral_, kd_ * derivative);
    prev_error_ = error;

    return output;

  }

  private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
};


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

  ros::Duration(3).sleep();
  sdk_ctrl_authority_service = nh.serviceClient<dji_osdk_ros::SDKControlAuthority>("dji_osdk_ros/sdk_control_authority");
  drone_task_service = nh.serviceClient<dji_osdk_ros::DroneTaskControl>("dji_osdk_ros/drone_task_control");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  bool land_result; 

  //!
  float32_t groundHeight = gps_position_.altitude;

  // Take off   
  dji_osdk_ros::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = dji_osdk_ros::DroneTaskControl::Request::TASK_TAKEOFF;
  drone_task_service.call(droneTaskControl);
  takeoff_result = droneTaskControl.response.result;
  if (takeoff_result){
    ROS_INFO("Drone taking off!");
  }
  ros::Duration(10).sleep();
  ROS_INFO("Take off done!");
  ROS_INFO("Takeoff Height: %f", gps_position_.altitude);

  // Save the commands requested in csv file
  string filename = "/home/brg/supernal/flight_control/Jun17_z_pid_cmd3.csv";
  writeCSVHeader(filename);
  // PIDTracking(dji_sdk_node, 0.0, 0.0, 10.0, 0.0, 0.3, groundHeight, filename);
  // PIDTracking(dji_sdk_node, 0.0, 0.0, 3.0, 0.0, 0.3, groundHeight, filename);
  // PIDTracking(dji_sdk_node, 0.0, 0.0, 3.0, 0.0, 0.3, groundHeight, filename);
  // PIDTracking(dji_sdk_node, 0.0, 0.0, 5.0, 0.0, 0.3, groundHeight, filename);

  PIDTracking(dji_sdk_node, 0.0, 0.0, 3.0, 0.0, 0.3, groundHeight, filename);
  PIDTracking(dji_sdk_node, -19.0, 0.0, 5.0, 0.0, 0.3, groundHeight, filename);
  PIDTracking(dji_sdk_node, 19.0, 0.0, 3.0, 0.0, 0.3, groundHeight, filename);

  // Land
  droneTaskControl.request.task = dji_osdk_ros::DroneTaskControl::Request::TASK_LAND;
  drone_task_service.call(droneTaskControl);
  land_result = droneTaskControl.response.result;
  if (land_result){
    ROS_INFO("Drone landing!");
  }
  ROS_INFO("Ground Height: %f", gps_position_.altitude);
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
                    float posThresholdInM, float yawThresholdInDeg, float zThresholdInM, float32_t groundHeight, string filename)
{
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_POSITION |
                       DJI::OSDK::Control::HORIZONTAL_POSITION |
                       DJI::OSDK::Control::YAW_ANGLE |
                       DJI::OSDK::Control::HORIZONTAL_GROUND |
                       DJI::OSDK::Control::STABLE_ENABLE);
  
  int responseTimeout = 1;
  int timeoutInMilSec = 40000;
  int controlFreqInHz = 50;  // Hz
  float cycleTime = 1.0 / controlFreqInHz;
  float cycleTimeInMs = cycleTime * 1000;
  int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
  int withinControlBoundsTimeReqmt = 1 * cycleTimeInMs;  // 100 cycles
  int elapsedTimeInMs = 0;
  int withinBoundsCounter = 0;
  int outOfBounds = 0;
  int brakeCounter = 0;
  double speedFactor = 1.0;
  double currentDistance = 0;
  double currentSpeed = 0;
  double expectedTime = 0;
  double numCycle = 0;
  double offsetPerCycleZ = 0;

  vector3f offsetDesired = {xSP, ySP, zSP};
  float yawDesiredInDeg = yawSP;
  double yawDesiredRad     = deg2rad * yawDesiredInDeg;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  vector3f positionCommand = {0, 0, 0};

  // Get original position and current position from GPS (RTK) data
  double currentRTKLat = (gps_position_.latitude)*deg2rad;
  double currentRTKLong = (gps_position_.longitude)*deg2rad;
  double currentHeight = gps_position_.altitude;
  double originalRTKLat = currentRTKLat;
  double originalRTKLong = currentRTKLong;
  double originalHeight = currentHeight;

  // Get initial offset. We will update this in a loop later
  vector3f localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, originalHeight);
  vector3f offsetRemaining = vector3FSub(offsetDesired, localoffset);
  quaternion currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
  double yawInRad = quaternionToEulerAngle(currentQuaternion).z;

  // Initialize position command
  positionCommand.x = (offsetDesired.x > 0) ? min(offsetDesired.x, speedFactor) : max(offsetDesired.x, -speedFactor);
  positionCommand.y = (offsetDesired.y > 0) ? min(offsetDesired.y, speedFactor) : max(offsetDesired.y, -speedFactor);
  positionCommand.z = originalHeight-groundHeight;

  double deltaZ =  offsetDesired.z - (originalHeight-groundHeight);
  double distanceXY = sqrt(pow(offsetDesired.x, 2) + pow(offsetDesired.y, 2));

  // Write a csv file

  while (elapsedTimeInMs < timeoutInMilSec) {
    ros::Time rosTime = ros::Time::now();
    double currentTime = rosTime.sec + rosTime.nsec / 1e9;
    DataRow row;
    row.time = currentTime;
    row.x = positionCommand.x;
    row.y = positionCommand.y;
    row.z = positionCommand.z;
    // writeCSVRow(filename, row);

    // Send the command to the aircraft
    dji_sdk_node_->flightControl(ctrl_flag, positionCommand.x, positionCommand.y, positionCommand.z, yawDesiredInDeg);
    usleep(cycleTimeInMs * 1000);

    elapsedTimeInMs += cycleTimeInMs;
    ROS_INFO("elapsedTimeInMs: %d", elapsedTimeInMs);
    
    // Update current position
    currentRTKLat = (gps_position_.latitude)*deg2rad;
    currentRTKLong = (gps_position_.longitude)*deg2rad;
    currentHeight = gps_position_.altitude;
    ROS_INFO("current position: %f, %f, %f", currentRTKLat, currentRTKLong, currentHeight-groundHeight);
    currentQuaternion = {attitude_data_.quaternion.w, attitude_data_.quaternion.x, attitude_data_.quaternion.y, attitude_data_.quaternion.z};
    yawInRad = quaternionToEulerAngle(currentQuaternion).z;

    // See how much farther we have to go
    localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, groundHeight);
    offsetRemaining = vector3FSub(offsetDesired, localoffset);
    ROS_INFO("offset remaining: %f, %f, %f, %f", offsetRemaining.x, offsetRemaining.y, offsetRemaining.z, vectorNorm(offsetRemaining));

    // See if we need to modify the setpoint
    if (fabs(offsetRemaining.x) < speedFactor) {
      positionCommand.x = offsetRemaining.x;
    }
    if (fabs(offsetRemaining.y) < speedFactor) {
      positionCommand.y = offsetRemaining.y;
    }

    // When the drone moves only along z-axis
    if (distanceXY == 0) {
      offsetPerCycleZ = (deltaZ > 0) ? 0.005 : -0.005;
    }

    // When the drone moves along x- and y-axes as well as z-axis
    else {
      currentDistance = sqrt(pow(localoffset.x, 2) + pow(localoffset.y, 2));
      currentSpeed = currentDistance / elapsedTimeInMs * 1000;
      expectedTime = distanceXY / currentSpeed;
      numCycle = expectedTime / cycleTime;
      offsetPerCycleZ = deltaZ / numCycle;
    }
    if (fabs(offsetRemaining.z) > zThresholdInM) {
      positionCommand.z += offsetPerCycleZ;
    }
    ROS_INFO("position command Z: %f", positionCommand.z);

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

bool generateSinusoidalTraj(DJISDKNode* dji_sdk_node_, float amplitude, float freq, float duration, float32_t originalHeight, string filename){
  int controlFreqInHz = 50;  // Hz
  float cycleTime = 1.0 / controlFreqInHz;  
  
  ros::Time rosTime  = ros::Time::now();
  double originTime = rosTime.sec + rosTime.nsec / 1e9;
  double currentTime = originTime;
  double elapsedTime = originTime - currentTime;
  DataRow row;
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_POSITION |
                       DJI::OSDK::Control::HORIZONTAL_POSITION |
                       DJI::OSDK::Control::YAW_ANGLE |
                       DJI::OSDK::Control::HORIZONTAL_GROUND |
                       DJI::OSDK::Control::STABLE_ENABLE);

  while (elapsedTime < duration) {
    rosTime  = ros::Time::now();
    currentTime = rosTime.sec + rosTime.nsec / 1e9;
    elapsedTime = currentTime - originTime;

    float zCmd = amplitude * sin(2 * C_PI * freq * elapsedTime) + originalHeight;
    ROS_INFO("time, zCmd: %f, %f", elapsedTime, zCmd);
    row.time = currentTime;
    row.x = 0.0;
    row.y = 0.0;
    row.z = zCmd;
    writeCSVRow(filename, row);
    dji_sdk_node_->flightControl(ctrl_flag, 0, 0, zCmd, 0.0);
    usleep(cycleTime * 1000000);
  }

  return true;
}

bool PIDTracking(DJISDKNode* dji_sdk_node_, 
                    float xSP, float ySP, float zSP, float yawSP, float speed, float32_t groundHeight, string filename)
{
  uint8_t ctrl_flag = (DJI::OSDK::Control::VERTICAL_POSITION |
                       DJI::OSDK::Control::HORIZONTAL_POSITION |
                       DJI::OSDK::Control::YAW_ANGLE |
                       DJI::OSDK::Control::HORIZONTAL_GROUND |
                       DJI::OSDK::Control::STABLE_ENABLE);
  
  int responseTimeout = 1;
  int timeoutInMilSec = 40000;
  int controlFreqInHz = 50;  // Hz
  float cycleTime = 1.0 / controlFreqInHz;
  float cycleTimeInMs = cycleTime * 1000;
  int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
  int withinControlBoundsTimeReqmt = 1 * cycleTimeInMs;  // 100 cycles
  int elapsedTimeInMs = 0;
  int withinBoundsCounter = 0;
  int outOfBounds = 0;
  int brakeCounter = 0;
  double speedFactor = 1.0;
  double currentDistance = 0;
  double currentSpeed = 0;
  double expectedTime = 0;
  double numCycle = 0;
  double offsetPerCycleZ = 0;
  double s = 0;

  float yawDesiredInDeg = yawSP;
  double yawDesiredRad     = deg2rad * yawDesiredInDeg;

  // Get original position and current position from GPS (RTK) data
  double originalRTKLat = (gps_position_.latitude)*deg2rad;
  double originalRTKLong = (gps_position_.longitude)*deg2rad;
  double originalHeight = gps_position_.altitude;
  double currentRTKLat = (gps_position_.latitude)*deg2rad;
  double currentRTKLong = (gps_position_.longitude)*deg2rad;
  double currentHeight = gps_position_.altitude;
  
  vector3f offsetDesired = {xSP, ySP, zSP-(originalHeight-groundHeight)};
  vector3f offsetRemaining = {0, 0, originalHeight-groundHeight};
  vector3f localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, groundHeight);
  vector3f positionCommand = {0, 0, originalHeight-groundHeight};

  double distance = vectorNorm(offsetDesired);
  double timeTaken = distance/speed;
  ros::Time rosTime = ros::Time::now();
  double startTime = rosTime.sec + rosTime.nsec / 1e9;
  double currentTime = startTime;

  double Ku_xy = 1.4;
  double Pu_xy = 1.2;
  double Ku_z = 3.38;
  double Pu_z = 1.8;
  double Kp_z = Ku_z / 1.7; // 0.8235
  double Ti_z = Pu_z / 2;   //0.9
  double Ki_z = Kp_z / Ti_z; //0.915
  double Td_z = Pu_z / 8; //0.225
  double Kd_z = Kp_z * Td_z + 2;

//Ku_xy / 1.7 = 0.8235
  double Kp_xy = Ku_xy/ 1.7; //0.8235
  double Ti_xy = Pu_xy/ 2;  //0.6
  double Ki_xy = Kp_xy / Ti_xy;//1.37
  double Td_xy = Pu_xy / 8;//0.15
  double Kd_xy = Kp_xy * Td_xy; //0.1235
  PID pid_x(Ku_xy/1.7, Pu_xy/2, Pu_xy/8);
  PID pid_y(Ku_xy/1.7, Pu_xy/2, Pu_xy/8);
  // PID pid_z(Ku_z/1.7, Pu_z/2, 10);

  // PID pid_x(Kp_xy, Ti_xy, Td_xy); 
  // PID pid_y(Kp_xy, Ti_xy, Kd_xy);
  // PID pid_z(Kp_z, Ki_z, Kd_z);
  PID pid_z(Kp_z, Ki_z, Kd_z);


  // PID pid_z(Ku_xy/1.7, Pu_xy/2, Pu_xy/8); 
  // PID pid_z(Ku_xy/1.7, Pu_xy/2, Pu_xy/8); 


  while (s < 1) {
    rosTime = ros::Time::now();
    currentTime = rosTime.sec + rosTime.nsec / 1e9;
    s = (currentTime - startTime) / timeTaken;
    
    double desired_x = s * xSP;
    double control_x = pid_x.calculate(desired_x, localoffset.x, cycleTime);
    positionCommand.x = control_x;
    // positionCommand.x = desired_x - localoffset.x;

    double desired_y = s * ySP;
    double control_y = pid_y.calculate(desired_y, localoffset.y, cycleTime);
    positionCommand.y = control_y;

    double desired_z = (1-s) * (originalHeight-groundHeight) + s * zSP;
    double control_z = pid_z.calculate(desired_z, currentHeight-groundHeight, cycleTime);
    positionCommand.z = desired_z + control_z;
    // positionCommand.z = originalHeight-groundHeight;

    DataRow row;
    row.time = currentTime;
    row.x = desired_x;
    row.y = desired_y;
    row.z = desired_z;
    row.x_input = control_x;
    row.y_input = control_y;
    row.z_input = control_z + desired_z;
    row.x_error = desired_x - localoffset.x;
    row.y_error = desired_y - localoffset.y;
    row.z_error = desired_z - (currentHeight - groundHeight);
    row.cur_minus_gnd_height = currentHeight - groundHeight;
    row.ground_height = groundHeight;

    writeCSVRow(filename, row);
    // ROS_INFO("position Command: %f, %f, %f", positionCommand.x, positionCommand.y, positionCommand.z);

    currentRTKLat = (gps_position_.latitude)*deg2rad;
    currentRTKLong = (gps_position_.longitude)*deg2rad;
    currentHeight = gps_position_.altitude;
    localoffset = localOffsetFromRTKOffset(currentRTKLat, currentRTKLong, originalRTKLat, originalRTKLong, currentHeight, groundHeight);
    // ROS_INFO("Local offset: %f, %f, %f", localoffset.x, localoffset.y, localoffset.z);

    dji_sdk_node_->flightControl(ctrl_flag, positionCommand.x, positionCommand.y, positionCommand.z, yawDesiredInDeg);
    usleep(cycleTimeInMs * 1000);
  }
  
  return true;
}