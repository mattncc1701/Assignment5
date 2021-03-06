#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <limits>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"

#include "compsci403_assignment5/CheckPointSrv.h"
#include "compsci403_assignment5/ObstaclePointCloudSrv.h"
#include "compsci403_assignment5/GetCommandVelSrv.h"
#include "compsci403_assignment5/GetTransformationSrv.h"
#include "compsci403_assignment5/PointCloudToLaserScanSrv.h"

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using std::cout;
using std::vector;
using std::max;
using std::min;

// GOOD TO KNOW COMMANDS FOR ACTUAL ROBOT.
/* 
export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH 
rosnode kill -a

look at max acceleration

*/
// Publisher for velocity command.
ros::Publisher velocity_command_publisher_;
ros::Publisher obstacle_point_cloud_publisher_;
ros::Publisher laserscan_cloud_publisher_;
ros::Publisher dynamic_window_cloud_publisher_;
ros::Publisher best_vw_publisher_;
ros::Publisher best_v_publisher_;
ros::Publisher point_cloud_publisher_;
ros::Publisher best_point_cloud_publisher_;

// Last received odometry message.
Odometry last_odometry;

// Last receieved transformation parameters
float R_transform[] = {1,0,0,0,1,0,0,0,1};
Point32 T_transform;

// INTRINSICS
const float px = 320;
const float py = 240;
const float fx = 588.446;
const float fy = -564.227;
const float a = 3.008;
const float b = -0.002745;

// ROBOT LIMITS
const float MAX_V = 0.5;
const float MAX_ROT = 1.5;
const float MAX_V_acc = 0.5;
const float MAX_ROT_acc = 2;
const float TIME_STEP = 0.05;

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPointToVector(const geometry_msgs::Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32.
geometry_msgs::Point32 ConvertVectorToPoint(const Vector3f& vector) {

  geometry_msgs::Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

// PART 1 Helper Function
void CheckPointUtility(Vector2f V, Vector2f P, float& free_path_length, bool& is_obstacle) {
  if (fabs(V.y()) > 0) {
    const float R = V.x() / V.y();
    Vector2f C(0, R)  ;
    if (fabs((P-C).norm() - fabs(R)) < 0.18f) {
      is_obstacle = true;
      float theta = (V.y() > 0) ? (atan2(P.x(), R - P.y())) : (atan2(P.x(), P.y() - R));
      free_path_length = max(0.0f, float(theta * fabs(R) - 0.18f));
    } else {
      free_path_length = 4.0;
      // free_path_length = std::numeric_limits<float>::max();
    }
  } else {
    if (fabs(P.y()) < 0.18f) {
      is_obstacle = true;
      free_path_length = max(0.0f, P.x() - 0.18f);
    } else {
      free_path_length = 4.0;
      // free_path_length = std::numeric_limits<float>::max();
    }
  }
}

bool CheckPointService(
    compsci403_assignment5::CheckPointSrv::Request& req,
    compsci403_assignment5::CheckPointSrv::Response& res) {
  // Observed point.
  const Vector2f P(req.P.x, req.P.y);
  // Desired velocity vector.
  const Vector2f V(req.v.x, req.w.z);
  bool is_obstacle = false;
  float free_path_length = 0.0;

  // Write code to compute is_obstacle and free_path_length.
  CheckPointUtility(V, P, free_path_length, is_obstacle);

  res.free_path_length = free_path_length;
  res.is_obstacle = is_obstacle;
  return true;
}

bool ObstaclePointCloudUtility(float R_req[], geometry_msgs::Point32 T_req, vector<Vector3f> point_cloud, vector<Vector3f>& obstacle_point_cloud) {

  // ROTATION
  Matrix3f R;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      R(row, col) = R_req[col * 3 + row];
    }
  }

  // TRANSLATION
  const Vector3f T(T_req.x, T_req.y, T_req.z);

  // Write code here to transform the input point cloud from the Kinect reference frame to the
  // robot's reference frame. Then filter out the points corresponding to ground
  // or heights larger than the height of the robot
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    Vector3f currPoint = point_cloud[i];

    // Perform Rotation, Followed by Translation.
    currPoint = currPoint.transpose() * R;
    currPoint = currPoint + T;

    // Check Min and Max z.
    if (currPoint.z() <= 0.0375 || currPoint.z() > .36) {
      continue;
    }

    // Insert Valid Filtered Point.
    (obstacle_point_cloud).insert((obstacle_point_cloud).end(), currPoint);
  }
  return true;
}

bool ObstaclePointCloudService(
    compsci403_assignment5::ObstaclePointCloudSrv::Request& req,
    compsci403_assignment5::ObstaclePointCloudSrv::Response& res) {

  // Copy over all the points.
  vector<Vector3f> point_cloud(req.P.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(req.P[i]);
  }

  // Convert float32 -> float
  float R[9];
  for(size_t i = 0; i < 9; i++){
    R[i] = req.R[i];
  }

  vector<Vector3f> filtered_point_cloud;
  // Write code here to transform the input point cloud from the Kinect reference frame to the
  // robot's reference frame. Then filter out the points corresponding to ground
  // or heights larger than the height of the robot
  ObstaclePointCloudUtility(R, req.T, point_cloud, filtered_point_cloud);

  res.P_prime.resize(filtered_point_cloud.size());
  for (size_t i = 0; i < filtered_point_cloud.size(); ++i) {
    res.P_prime[i] = ConvertVectorToPoint(filtered_point_cloud[i]);
  }
  return true;
}

sensor_msgs::LaserScan PointCloudToLaserScanUtility(vector<Vector3f> point_cloud, vector<float>& ranges){
  // Set all of the ranges to be inifintely far away
  for (int i = 0; i < 56; ++i) {
    ranges.insert(ranges.begin(), 4.0);
  }

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    Vector3f temp = point_cloud[i];
    // Project point to the ground plan
    temp.z() = 0;
    // Calculate the points angle in relation to the bot
    float theta = atan2(-temp.y(), temp.x());
    // .488692 == 28 degress in radians. Removes outliers
    if (theta > .488692 || theta < -.488692) {
      continue;
    }
    // Calculate the points distance from the robot
    float distance = sqrt(pow(temp.x(),2) + pow(temp.y(),2));
    int index = int(theta*360/(2*M_PI)) + 28;
    // If the point is closer than the point currently in that angles position update it
    if (distance < ranges[index]) {
      ranges[index] = distance;
    } 
  }
  sensor_msgs::LaserScan laser_scan = sensor_msgs::LaserScan();
  laser_scan.ranges = ranges;
  laser_scan.angle_increment = float(float((1.0/360.0))*2.0*M_PI); // One degree
  laser_scan.angle_max = float(float((28.0/360.0))*2.0*M_PI); // 28 degrees
  laser_scan.angle_min = float(float((-28.0/360.0))*2.0*M_PI); // -28 degrees
  laser_scan.range_min = 0.8; //.8 meters, min distance
  laser_scan.range_max = 4; //4 meters, max distance

  return laser_scan;
}

bool PointCloudToLaserScanService(
    compsci403_assignment5::PointCloudToLaserScanSrv::Request& req,
    compsci403_assignment5::PointCloudToLaserScanSrv::Response& res) {

  // Copy over all the points.
  vector<Vector3f> point_cloud(req.P.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(req.P[i]);
  }

  vector<float> ranges;
  // Process the point cloud here and convert it to a laser scan
  PointCloudToLaserScanUtility(point_cloud, ranges);
  
  res.ranges = ranges;
  return true;
}

float velocity(float v,float w){
  return v;
}

float angle(float v, float w){
   return fabs(1/w);
}

bool GetCommandVelUtility(Vector2f V, sensor_msgs::Image Image, const bool is_part5, Vector2f& new_velocity) {
  vector<Vector3f> point_cloud;
  // The input v0 and w0 are each vectors. The x component of v0 is the linear 
  // velocity towards forward direction and the z component of w0 is the
  // rotational velocity around the z axis, i.e. around the center of rotation
  // of the robot and in counter-clockwise direction

  for (unsigned int y = 0; y < Image.height; ++y) {
    for (unsigned int x = 0; x < Image.width; ++x) {
      // Add code here to only process only every nth pixel
      if (((x+y) % 10) != 0) { // assuming n = 10 like it says in part 4
        continue;
      }      

      uint16_t byte0 = Image.data[2 * (x + y * Image.width) + 0];
      uint16_t byte1 = Image.data[2 * (x + y * Image.width) + 1];
      if (!Image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 12 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
      // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics and add it to your point cloud.
      Vector3f point;

      float depth =  1.0 / (a + (b*raw_depth));
      float Z =  depth;
      float X = ((x - px)*1.0 / fx ) * Z;
      float Y = ((y - py)*1.0 / fy ) * Z;

      point.x() = X;
      point.y() = Y;
      point.z() = Z;

      // ROTATE BY y AXIS PI/2
      if(is_part5){
        X = 0 + 0 + point.z();
        Y = 0 + point.y() + 0;
        Z = -point.x() + 0 + 0;

        point.x() = X;
        point.y() = Y;
        point.z() = Z;
      } 
      point = Vector3f(X,Y,Z);

      point_cloud.push_back(point);
    }
  }
  vector<Vector3f> obstacle_point_cloud;
  // Part 5 requires us to transform the pointcloud to the robots reference frame
  if (is_part5) {
    ObstaclePointCloudUtility(R_transform, T_transform, point_cloud, obstacle_point_cloud);
  } else {
    obstacle_point_cloud = point_cloud;
  }

  // TEST PART 5: PUBLISH NEW FILTERED POINT CLOUD.
  sensor_msgs::PointCloud point_cloud_result;
  point_cloud_result.header = Image.header;
  for (size_t i = 0; i < obstacle_point_cloud.size(); ++i) {
    point_cloud_result.points.insert(point_cloud_result.points.end(), ConvertVectorToPoint(obstacle_point_cloud[i]));
  }
  best_point_cloud_publisher_.publish(point_cloud_result);

  
  // Use your code from part 3 to convert the point cloud to a laser scan
  vector<float> ranges;
  // I changed the utility function to return a LaserScan object to make our lives easier
  sensor_msgs::LaserScan laser_scan = PointCloudToLaserScanUtility(obstacle_point_cloud, ranges);
  laser_scan.header = Image.header;
  laserscan_cloud_publisher_.publish(laser_scan);

  // Implement dynamic windowing approach to find the best velocity command for next time step
  float V_min = max(float(0.0), V.x() - (MAX_V_acc * TIME_STEP));
  float V_max = min(MAX_V, V.x() + (MAX_V_acc * TIME_STEP));
  
  // Rotation: Min(Smallest Rotation Velocity, W - (W_a * Timestep))
  float W_min = max(-MAX_ROT, V.y() - (MAX_ROT_acc * TIME_STEP));
  float W_max = min(MAX_ROT, V.y() + (MAX_ROT_acc * TIME_STEP));

  int V_WINDOW_SIZE = 21;
  int W_WINDOW_SIZE = 40;

  float V_increment = (V_max-V_min)/V_WINDOW_SIZE;
  float W_increment = (W_max-W_min)/W_WINDOW_SIZE;

  // COST PARAMETERS (FROM PDF ON DYNAMIC WINDOWS).
  float alpha = 0.2;
  float beta = 2.0;
  float gamma = 1.0;

  // BEST V,W
  Vector2f bestVW = Vector2f(0,0);
  float bestCost = std::numeric_limits<float>::max();

  float v = V_min;
  // ITERATE THROUGH ALL V,W PAIRS.
  for(int i = 0; i < V_WINDOW_SIZE; i++){
    float w = W_min;
    for(int j = 0; j < W_WINDOW_SIZE; j++){
      // Go through the laser scan (our obstacles) and see if any make the current V,W pair
      // inadmissible (i.e. they are in the robots path if it goes that direction).
      bool admissible = true;
      float min_distance = std::numeric_limits<float>::max();;

      for(size_t k = 0; k < ranges.size(); k++){
        Vector2f V_temp(v, w);
        float current_theta = 2.0 * M_PI * (float(k) - 28.0)/360.0;
        // Calculate the x,y cooridnates of the point in the laser scan
        // NOTE for some reason I think its suppose to be cos = x and sin = y but I'm getting confused
        // Vector2f P(sin(current_theta) * ranges[k], cos(current_theta) * ranges[k]);
        Vector2f P(cos(current_theta) * ranges[k], sin(current_theta) * ranges[k]);

        float free_path_length = 0.0;
        bool is_obstacle = false; 
        // This will tell us whether the current point in the laser scan we are looking at 
        // makes the V,W pair inadmissible.
        CheckPointUtility(V_temp, P, free_path_length, is_obstacle);

        // if (is_obstacle) {
        //   // V is not an admissible speed because there is an obstacle in the way
        //   admissible = false;
        //   break;
        // }

        // Check for stopping distance
        float stopping_distance = pow(v,2)/(2*MAX_V_acc);
        /*
         * The free_path_length is the distance we are going to travel
         * so if the stopping distance isn't smaller than our free_path_length
         * then we won't be have enough time to stop.
         */
        if(stopping_distance > free_path_length){
          admissible = false;
                break;
        }

        if(min_distance > (free_path_length - stopping_distance)){
          min_distance = (free_path_length - stopping_distance);
        }

      }

      if(!admissible){
        continue;
      }

      // Limit to range finder MAX.
      if (min_distance > 4) {
        min_distance = 4;
      }

      // Calculate Cost
      // float currCost = alpha*fabs(w) + beta/min_distance + gamma/velocity(v,w);
      float currCost = alpha*fabs(w) - beta*min_distance - gamma*velocity(v,w);

      // Update best cost
      if(currCost < bestCost){
        // cout << "Best Cost: " << "Distance: " << beta/min_distance << " Angle: " << alpha*fabs(w) << "  Velocity : " << gamma/velocity(v,w) << "\n";
        // cout << currCost << " \n";
        bestCost = currCost;
        bestVW = Vector2f(v,w);
      }

      w += W_increment;
    }
    v += V_increment;
  }
  // cout << "::FINAL ANSWER:: \n";

  // MARKER for best W.
  visualization_msgs::Marker marker; 
  marker.header.frame_id = "kinect_0";
  marker.header.stamp = Image.header.stamp;
  marker.scale.x = 0.1; 
  marker.scale.y = 0.2; 
  marker.color.a = 0.5; 
  marker.color.r = 255; 
  marker.type = marker.ARROW; 
  marker.action = 0; 
  geometry_msgs::Point point1; 
  point1.x = 0; 
  point1.y = 0;
  point1.z = 0; 
  geometry_msgs::Point point2;
  point2.x = 1;
  point2.y = bestVW.y(); // HERE 
  point2.z = 0; 
  marker.points.resize(2);
  marker.points[0] = point1;
  marker.points[1] = point2;
  best_vw_publisher_.publish(marker);

  // MARKER for best V.
  visualization_msgs::Marker best_v; 
  best_v.header.frame_id = "kinect_0";
  best_v.header.stamp = Image.header.stamp;
  best_v.scale.x = 0.1; 
  best_v.scale.y = 0.2; 
  best_v.color.a = 0.5; 
  best_v.color.r = 255; 
  best_v.type = marker.ARROW; 
  best_v.action = 0; 
  geometry_msgs::Point pointV;
  pointV.x = bestVW.x(); // HERE
  pointV.y = 0; 
  pointV.z = 0; 
  best_v.points.resize(2);
  best_v.points[0] = point1;
  best_v.points[1] = pointV;
  best_v_publisher_.publish(best_v);

  new_velocity = bestVW;

  return true;
}

bool GetCommandVelService(
    compsci403_assignment5::GetCommandVelSrv::Request& req,
    compsci403_assignment5::GetCommandVelSrv::Response& res) {

  const Vector2f V(req.v0.x, req.w0.z);
  Vector2f new_velocity(0,0);
  GetCommandVelUtility(V, req.Image, false, new_velocity);
  // Return the best velocity command
  // Cv is of type Point32 and its x component is the linear velocity towards forward direction
  // you do not need to fill its other components
  res.Cv.x = new_velocity.x();
  // Cw is of type Point32 and its z component is the rotational velocity around z axis
  // you do not need to fill its other components
  res.Cw.z = new_velocity.y();

  return true;
}

void OdometryCallback(const nav_msgs::Odometry& odometry) {
  last_odometry = odometry;
}

void DepthImageCallback(const sensor_msgs::Image& depth_image) {
  Twist command_vel;
  // Current velocity
  const float v0 = last_odometry.twist.twist.linear.x;
  const float w0 = last_odometry.twist.twist.angular.z;

  // Use your code from all other parts to process the depth image, 
  // find the best velocity command and publish the velocity command

  const Vector2f V(v0, w0);
  Vector2f new_velocity(0,0);
  GetCommandVelUtility(V, depth_image, true, new_velocity);

  command_vel.linear.x = new_velocity.x();
  command_vel.angular.z = new_velocity.y();

  cout <<" (V,W): " << new_velocity.x() << " " << new_velocity.y() << "\n";


  velocity_command_publisher_.publish(command_vel);

  // This was only used for testing.
  // PUBLISH IMAGE AS POINT CLOUD
  // sensor_msgs::PointCloud point_cloud;
  // point_cloud.header = depth_image.header;

  // for (unsigned int y = 0; y < depth_image.height; ++y) {
  //   for (unsigned int x = 0; x < depth_image.width; ++x) {
  //     uint16_t byte0 = depth_image.data[2 * (x + y * depth_image.width) + 0];
  //     uint16_t byte1 = depth_image.data[2 * (x + y * depth_image.width) + 1];
  //     if (!depth_image.is_bigendian) {
  //       std::swap(byte0, byte1);
  //     }
  //     // Combine the two bytes to form a 16 bit value, and disregard the
  //     // most significant 4 bits to extract the lowest 12 bits.
  //     const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
  //     // Reconstruct 3D point from x, y, raw_depth.
  //     geometry_msgs::Point32 point;
  //     // Modify the following lines of code to do the right thing, with the
  //     // correct parameters.
  //     float depth =  1.0 / (a + (b*raw_depth));
  //     float Z =  depth;
  //     float X = ((x - px)*1.0 / fx ) * Z;
  //     float Y = ((y - py)*1.0 / fy ) * Z;

  //     point.x = X;
  //     point.y = Y;
  //     point.z = Z;

  //     // ROTATE BY y AXIS PI/2
  //     X = 0 + 0 + point.z;
  //     Y = 0 + point.y + 0;
  //     Z = -point.x + 0 + 0;

  //     point.x = X;
  //     point.y = Y;
  //     point.z = Z;

  //     point_cloud.points.push_back(point);
  //   }
  // }
  // point_cloud_publisher_.publish(point_cloud);

}

// Part 2: Test
void FilteredPointCloudCallback(const sensor_msgs::PointCloud& point_cloud) {

  // Preset Values.
  float R[] = {1,0,0,0,1,0,0,0,1}; // NO ROTATION
  geometry_msgs::Point32 T; // DEFAULT TRANSLATION
  T.x = 0.13; 
  T.y = 0; 
  T.z = 0.305; 
  vector<Vector3f> points(point_cloud.points.size()); // GIVEN POINT CLOUD
  vector<Vector3f> filtered_point_cloud;  // OBSTACLE POINT CLOUD
  sensor_msgs::PointCloud point_cloud_result = sensor_msgs::PointCloud();
  vector<geometry_msgs::Point32> vals;  // Vector3f -> Point32 vector.

  // Initialize points and header on point cloud.
  point_cloud_result.points = vals;
  point_cloud_result.header = point_cloud.header;

  // Create vector of Vector3f from given cloud.
  for (size_t i = 0; i < points.size(); ++i) {
    points[i] = ConvertPointToVector(point_cloud.points[i]);
  }

  // Create Obstacle Point Cloud (inserts vals to filtered_point_cloud by reference).
  ObstaclePointCloudUtility(R,T,points,filtered_point_cloud);

  // Insert all filtered_point_cloud. 
  for (size_t i = 0; i < filtered_point_cloud.size(); ++i) {
    point_cloud_result.points.insert(point_cloud_result.points.end(), ConvertVectorToPoint(filtered_point_cloud[i]));
  }

  obstacle_point_cloud_publisher_.publish(point_cloud_result);
}

// Part 3: Test
void LaserScanCallback(const sensor_msgs::PointCloud& point_cloud) {

  // Preset Values.
  vector<Vector3f> points(point_cloud.points.size()); // GIVEN POINT CLOUD
  vector<float> ranges;  // Ranges for laser scan

  // Generating a point cloud (optional argument)
  vector<Vector3f> filtered_point_cloud;  // LASER POINT CLOUD
  sensor_msgs::PointCloud point_cloud_result = sensor_msgs::PointCloud();
  vector<geometry_msgs::Point32> vals;  // Vector3f -> Point32 vector.

  // Create vector of Vector3f from given cloud.
  for (size_t i = 0; i < points.size(); ++i) {
    points[i] = ConvertPointToVector(point_cloud.points[i]);
  }

  // PointCloudToLaserScanUtility(points, ranges);

  sensor_msgs::LaserScan laser_scan = PointCloudToLaserScanUtility(points, ranges);
  laser_scan.header = point_cloud.header;

  laserscan_cloud_publisher_.publish(laser_scan);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_assignment5");
  ros::NodeHandle n;

  ros::ServiceServer service1 = n.advertiseService(
      "/COMPSCI403/CheckPoint", CheckPointService);
  ros::ServiceServer service2 = n.advertiseService(
      "/COMPSCI403/ObstaclePointCloud", ObstaclePointCloudService);
  ros::ServiceServer service3 = n.advertiseService(
      "/COMPSCI403/PointCloudToLaserScan", PointCloudToLaserScanService);
  ros::ServiceServer service4 = n.advertiseService(
      "/COMPSCI403/GetCommandVel", GetCommandVelService);

  // This code will retrieve the transformation given by the service GetTransformation
  ros::ServiceClient client = n.serviceClient<compsci403_assignment5::GetTransformationSrv>("GetTransformation");
  compsci403_assignment5::GetTransformationSrv srv;
  if (client.call(srv))
  {
    for(size_t i = 0; i < 9; i++){
      R_transform[i] = srv.response.R[i];
    }
    T_transform = srv.response.T;
  }else{
    T_transform.x = 0.13; 
    T_transform.y = 0; 
    T_transform.z = 0.305; 
  }

  velocity_command_publisher_ =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  // PART 2: TEST PUBLISHER FOR FILTERING POINT CLOUD 
  obstacle_point_cloud_publisher_ =
      n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/ObstaclePointCloud", 1);

  // PART 3: TEST PUBLISHER FOR LASER SCAN 
  laserscan_cloud_publisher_ =
      n.advertise<sensor_msgs::LaserScan>("/COMPSCI403/LaserScan", 1);

  // PART 5: TEST PUBLISHER FOR DYNAMIC WINDOW 
  dynamic_window_cloud_publisher_ =
      n.advertise<sensor_msgs::Image>("/COMPSCI403/DynamicWindow", 1);

  best_vw_publisher_ =
      n.advertise<visualization_msgs::Marker>("/COMPSCI403/BestVW", 1);

  best_v_publisher_ =
      n.advertise<visualization_msgs::Marker>("/COMPSCI403/Part5BestV", 1);

  // PART 5: TEST BEST POINT CLOUD.
  best_point_cloud_publisher_ =
      n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/Part5PointCloud", 1);

  // PART 5: TEST PUBLISHER FOR POINT CLOUD ASSIGNMENT 5 BAG 
  point_cloud_publisher_ =
      n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/BAG_POINT_CLOUD", 1);


  ros::Subscriber depth_image_subscriber =
      n.subscribe("/Cobot/Kinect/Depth", 1, DepthImageCallback);
  ros::Subscriber odometry_subscriber =
      n.subscribe("/odom", 1, OdometryCallback);

  // // PART 2: Subscribed to FilteredPointCloud
  ros::Subscriber filtered_point_cloud_subscriber =
      n.subscribe("/Cobot/Kinect/FilteredPointCloud", 1, FilteredPointCloudCallback);
  // // PART 3: Subscribed to FilteredPointCloud. doing stuff with lasers.
  ros::Subscriber laserscan_cloud_subscriber =
      n.subscribe("/Cobot/Kinect/FilteredPointCloud", 1, LaserScanCallback);

  ros::spin();


  return 0;
}










