#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "first_project/CalculateDistance.h"
#include "first_project/CollisionStatus.h"
#include "first_project/CollisionThresholdsConfig.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

bool isOdometryValid(const nav_msgs::OdometryConstPtr& odo1)
{
  return odo1->pose.pose.position.x != NAN && odo1->pose.pose.position.y != NAN && odo1->pose.pose.position.z != NAN;
}

class CollisionStatus
{
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> ApproxPolicy;

public:
  CollisionStatus()
    : n()
    , client(n.serviceClient<first_project::CalculateDistance>("calculate_distance"))
    , pub_status(n.advertise<first_project::CollisionStatus>("collision_status_topic", 1000))
    , odo_1_sub(n, "odo_1", 1)
    , odo_2_sub(n, "odo_2", 1)
    , sync(ApproxPolicy(10), odo_1_sub, odo_2_sub)
    , dr_server()
  {
    sync.registerCallback(boost::bind(&CollisionStatus::distanceCallback, this, _1, _2));
    dr_server.setCallback(boost::bind(&CollisionStatus::dynamicReconfigCallback, this, _1, _2));
  }

private:
  void distanceCallback(const nav_msgs::OdometryConstPtr& odo1, const nav_msgs::OdometryConstPtr& odo2)
  {
    if (isOdometryValid(odo1) && isOdometryValid(odo2))
    {
      first_project::CalculateDistance srv;

      srv.request.point1 = odo1->pose.pose.position;
      srv.request.point2 = odo2->pose.pose.position;

      // ros::service::waitForService(client.getService());
      client.waitForExistence();
      if (client.call(srv))
      {
        first_project::CollisionStatus msg;

        msg.header.stamp = ros::Time::now();
        msg.distance = srv.response.distance;

        if (msg.distance <= crash_threshold)
        {
          msg.status = msg.STATUS_CRASH;
        }
        else if (msg.distance <= unsafe_threshold || msg.distance == NAN)
        {
          msg.status = msg.STATUS_UNSAFE;
        }
        else
        {
          msg.status = msg.STATUS_SAFE;
        }

        pub_status.publish(msg);
      }
      else
      {
        ROS_ERROR("Failed to call distance service");
      }
    }
  }

  void dynamicReconfigCallback(first_project::CollisionThresholdsConfig& config, uint32_t level)
  {
    ROS_INFO("Reconfigure Request: crash_t: %f,  unsafe_t: %f, level: %d", config.crash_threshold,
             config.unsafe_threshold, level);

    crash_threshold = config.crash_threshold;
    unsafe_threshold = config.unsafe_threshold;
  }

  ros::NodeHandle n;
  ros::ServiceClient client;
  ros::Publisher pub_status;

  message_filters::Subscriber<nav_msgs::Odometry> odo_1_sub;
  message_filters::Subscriber<nav_msgs::Odometry> odo_2_sub;
  message_filters::Synchronizer<ApproxPolicy> sync;

  dynamic_reconfigure::Server<first_project::CollisionThresholdsConfig> dr_server;

  // Default values, but dynamicReconfigCallback gets called on startup with the real values.
  float crash_threshold = 1;
  float unsafe_threshold = 5;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_status_node");

  CollisionStatus status{};
  ROS_INFO("Collision status node running.");


  ros::spin();

  return 0;
}