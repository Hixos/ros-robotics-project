
#include <cmath>
#include "first_project/CalculateDistance.h"
#include "ros/ros.h"

bool distance(first_project::CalculateDistance::Request &req, first_project::CalculateDistance::Response &res)
{
  res.distance = sqrtf(powf(req.point1.x - req.point2.x, 2) + powf(req.point1.y - req.point2.y, 2) +
                       powf(req.point1.z - req.point2.z, 2));
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_distance", distance);

  ROS_INFO("Distance service running.");
  ros::spin();

  return 0;
}