#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <limits>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

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


// Publisher for velocity command.
ros::Publisher velocity_command_publisher_;
ros::Publisher obstacle_point_cloud_publisher_;

// Last received odometry message.
Odometry last_odometry;

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

void testCheckPoint(Vector2f V, Vector2f P) {

  bool is_obstacle = false;
  float free_path_length = 0.0;
  if (fabs(V.y()) > 0) {
    const float R = V.x() / V.y();
    Vector2f C(0, R);
    if (fabs((P-C).norm() - fabs(R)) < 0.18f) {
      is_obstacle = true;
      float theta = (V.y() > 0) ? (atan2(P.x(), R - P.y())) : (atan2(P.x(), P.y() - R));
      free_path_length = max(0.0f, float(theta * fabs(R) - 0.18f));
    } else {
      free_path_length = std::numeric_limits<float>::max();
    }
  } else {
    if (fabs(P.y()) < 0.18f) {
      is_obstacle = true;
      free_path_length = max(0.0f, P.x() - 0.18f);
    } else {
      free_path_length = std::numeric_limits<float>::max();
    }
  }
  cout << free_path_length;
  cout << "\n";
  cout << is_obstacle;
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
  if (fabs(V.y()) > 0) {
    const float R = V.x() / V.y();
    Vector2f C(0, R);
    if (fabs((P-C).norm() - fabs(R)) < 0.18f) {
      is_obstacle = true;
      float theta = (V.y() > 0) ? (atan2(P.x(), R - P.y())) : (atan2(P.x(), P.y() - R));
      free_path_length = max(0.0f, float(theta * fabs(R) - 0.18f));
    } else {
      free_path_length = std::numeric_limits<float>::max();
    }
  } else {
    if (fabs(P.y()) < 0.18f) {
      is_obstacle = true;
      free_path_length = max(0.0f, P.x() - 0.18f);
    } else {
      free_path_length = std::numeric_limits<float>::max();
    }
  }

  res.free_path_length = free_path_length;
  res.is_obstacle = is_obstacle;
  return true;
}


// bool ObstaclePointCloudUtility(float32[] R, Point32 T, Point32[] P, Point32[] P_prime) {
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
    if (currPoint.z() <= 0 || currPoint.z() > .36) {
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
  // Set all of the ranges to be inifintely far away
  for (int i = 0; i < 56; ++i) {
    ranges.insert(ranges.begin(), std::numeric_limits<float>::max());
  }

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    Vector3f temp = point_cloud[i];
    // Project point to the ground plan
    temp.z() = 0;
    // Calculate the points angle in relation to the bot
    int theta = int(atan2(temp.y(), temp.x()));
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

  res.ranges = ranges;
  return true;
}

bool GetCommandVelService(
    compsci403_assignment5::GetCommandVelSrv::Request& req,
    compsci403_assignment5::GetCommandVelSrv::Response& res) {

  vector<Vector3f> point_cloud;
  // The input v0 and w0 are each vectors. The x component of v0 is the linear 
  // velocity towards forward direction and the z component of w0 is the
  // rotational velocity around the z axis, i.e. around the center of rotation
  // of the robot and in counter-clockwise direction
  const Vector2f V(req.v0.x, req.w0.z);

  for (unsigned int y = 0; y < req.Image.height; ++y) {
    for (unsigned int x = 0; x < req.Image.width; ++x) {
      // Add code here to only process only every nth pixel

      uint16_t byte0 = req.Image.data[2 * (x + y * req.Image.width) + 0];
      uint16_t byte1 = req.Image.data[2 * (x + y * req.Image.width) + 1];
      if (!req.Image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 12 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
      // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics and add it to your point cloud.
      Vector3f point;

      point_cloud.push_back(point);
    }
  }

  // Use your code from part 3 to convert the point cloud to a laser scan

  // Implement dynamic windowing approach to find the best velocity command for next time step

  // Return the best velocity command
  // Cv is of type Point32 and its x component is the linear velocity towards forward direction
  // you do not need to fill its other components
  res.Cv.x = 0;
  // Cw is of type Point32 and its z component is the rotational velocity around z axis
  // you do not need to fill its other components
  res.Cw.z = 0;

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

  command_vel.linear.x = 0; // replace with your calculated linear velocity c_v
  command_vel.angular.z = 0; // replace with your angular calculated velocity c_w
  velocity_command_publisher_.publish(command_vel);
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

  velocity_command_publisher_ =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  // PART 2: TEST PUBLISHER FOR FILTERING POINT CLOUD 
  obstacle_point_cloud_publisher_ =
      n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/ObstaclePointCloud", 1);

  ros::Subscriber depth_image_subscriber =
      n.subscribe("/Cobot/Kinect/Depth", 1, DepthImageCallback);
  ros::Subscriber odometry_subscriber =
      n.subscribe("/odom", 1, OdometryCallback);

  // PART 2: Subscribed to FilteredPointCloud
  ros::Subscriber filtered_point_cloud_subscriber =
      n.subscribe("/Cobot/Kinect/FilteredPointCloud", 1, FilteredPointCloudCallback);
  ros::spin();


  return 0;
}









