#include <math.h>
#include <sstream>
#include <string>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "tf/transform_broadcaster.h"

using std::string;

class LLA2ENU
{
public:
  LLA2ENU(ros::NodeHandle& n, string tf_name, float latitude_init, float longitude_init, float h0)
    : n(n), tf_name(tf_name), latitude_init(latitude_init), longitude_init(longitude_init), h0(h0)
  {
    // Advertise the topic & subscribe to gps data
    pub_enu = n.advertise<nav_msgs::Odometry>("output_odo", 1000);
    sub_lla = n.subscribe("input", 1000, &LLA2ENU::navSatFixCallback, this);
  }

  void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    nav_msgs::Odometry odo = lla2enu(msg);

    // Assign the same stamp as the gps data
    odo.header.stamp = msg->header.stamp;
    odo.header.frame_id = "world";

    // Publish odometry unconditionally (may contain NaN)
    pub_enu.publish(odo);

    // Publish TF only if we have valid odometry
    if (odo.pose.pose.position.x != NAN && odo.pose.pose.position.y != NAN && odo.pose.pose.position.z != NAN)
    {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z));
      tf::Quaternion q;
      q.setRPY(0, 0, 0);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", tf_name));
    }
  }

private:
  nav_msgs::Odometry lla2enu(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    nav_msgs::Odometry odo;
    if (msg->latitude == 0 || msg->latitude == 0 || msg->altitude == 0)
    {
      // If any of the GPS data is not available, set all coordinates to NaN since we don't have  a reliable position
      odo.pose.pose.position.x = NAN;
      odo.pose.pose.position.y = NAN;
      odo.pose.pose.position.z = NAN;
    }
    else
    {
      // fixed values
      double a = 6378137;
      double b = 6356752.3142;
      double f = (a - b) / a;
      double e_sq = f * (2 - f);
      float deg_to_rad = 0.0174533;

      // input data from msg
      float latitude = msg->latitude;
      float longitude = msg->longitude;
      float h = msg->altitude;

      // lla to ecef
      float lamb = deg_to_rad * (latitude);
      float phi = deg_to_rad * (longitude);
      float s = sin(lamb);
      float N = a / sqrt(1 - e_sq * s * s);

      float sin_lambda = sin(lamb);
      float cos_lambda = cos(lamb);
      float sin_phi = sin(phi);
      float cos_phi = cos(phi);

      float x = (h + N) * cos_lambda * cos_phi;
      float y = (h + N) * cos_lambda * sin_phi;
      float z = (h + (1 - e_sq) * N) * sin_lambda;

      // ecef to enu

      lamb = deg_to_rad * (latitude_init);
      phi = deg_to_rad * (longitude_init);
      s = sin(lamb);
      N = a / sqrt(1 - e_sq * s * s);

      sin_lambda = sin(lamb);
      cos_lambda = cos(lamb);
      sin_phi = sin(phi);
      cos_phi = cos(phi);

      float x0 = (h0 + N) * cos_lambda * cos_phi;
      float y0 = (h0 + N) * cos_lambda * sin_phi;
      float z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

      float xd = x - x0;
      float yd = y - y0;
      float zd = z - z0;

      float xEast = -sin_phi * xd + cos_phi * yd;
      float yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
      float zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

      odo.pose.pose.position.x = xEast;
      odo.pose.pose.position.y = yNorth;
      odo.pose.pose.position.z = zUp;
    }
    return odo;
  }

  float latitude_init;
  float longitude_init;
  float h0;

  string tf_name;

  ros::NodeHandle& n;
  ros::Subscriber sub_lla;
  ros::Publisher pub_enu;
  tf::TransformBroadcaster br;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lla2enu_node");
  ros::NodeHandle n;
  ros::NodeHandle private_node_handle("~");

  float latitude_init, longitude_init, h0;
  string tf_name;

  // Retrieve parameters
  if (!private_node_handle.getParam("tf_name", tf_name))
  {
    // Set default value
    ROS_ERROR("Could not retrieve parameter 'tf_name'");
    return 1;
  }

  if (!n.getParam("enu/latitude_init", latitude_init))
  {
    // Set default value
    latitude_init = 45.6216561271;

    ROS_ERROR("Could not retrieve parameter 'enu/latitude_init'");
  }

  if (!n.getParam("enu/longitude_init", longitude_init))
  {
    // Set default value
    longitude_init = 9.28155230131;

    ROS_ERROR("Could not retrieve parameter 'enu/longitude_init'");
  }

  if (!n.getParam("enu/altitude_init", h0))
  {
    // Set default value
    h0 = 224.616616895;

    ROS_ERROR("Could not retrieve parameter 'enu/altitude_init'");
  }

  LLA2ENU lla2enu{ n, tf_name, latitude_init, longitude_init, h0 };

  ROS_INFO("lla2enu node running.");

  ros::spin();

  return 0;
}
