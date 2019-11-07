#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <global_planner/planner_core.h>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int8.h"

bool getbaselinkpose(tf::Stamped<tf::Pose>& odom_pose,double& x, double& y, double& yaw,
                               const ros::Time& t){
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
  tf::Vector3(0,0,0)), ros::Time::now(), "base_link" );
 tf::StampedTransform transform;
 tf::TransformListener* tf1_;
  try
  {
  tf1_ = new tf::TransformListener();
  //tf1_->transformPose("map", ident, odom_pose);
  tf1_->waitForTransform("odom", "base_link",ros::Time(0), ros::Duration(3.0));
  tf1_->lookupTransform("odom","base_link",ros::Time(0),transform);
  }
  catch(tf::TransformException e)
  {
  ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
  return false;
  }

 x = transform.getOrigin().x();
 y = transform.getOrigin().y();
 double pitch,roll;
 odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);
 /* x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);
*/
  return true;}


void fillPathRequest(nav_msgs::GetPlan::Request &request)
{
  tf::Stamped<tf::Pose> odom_pose;
  double x, y, yaw;
  ros::Time t=ros::Time::now();
  getbaselinkpose(odom_pose,x,y,yaw,t);
  ROS_INFO("x=%lf,y=%lf",x,y);
request.start.header.frame_id ="map";
request.start.pose.position.x = x;//初始位置x坐标
request.start.pose.position.y = y;//初始位置y坐标
request.start.pose.orientation.w = yaw;//方向
request.goal.header.frame_id = "map";
request.goal.pose.position.x = 0.8;//终点坐标
request.goal.pose.position.y = 3;
request.goal.pose.orientation.w = 1.0;
request.tolerance = 0.5;//如果不能到达目标，最近可到的约束
}
//路线规划结果回调
void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
{

      if (serviceClient.call(srv)) {

      if (!srv.response.plan.poses.empty()) {
      //forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses)
        for(size_t i = 1; i < (srv.response.plan.poses.size()-1); i++)
        {

      ROS_INFO("x = %f, y = %f", srv.response.plan.poses[i].pose.position.x, srv.response.plan.poses[i].pose.position.y);
      }
      }
      else {
      ROS_WARN("Got empty plan");
      }
      }
      else {
       ROS_ERROR("Failed to call service %s - is the robot moving?",
      serviceClient.getService().c_str());
      }
}


int main(int argc, char** argv)
{
ros::init(argc, argv, "fun_node");
ros::NodeHandle nh;
std::string service_name = "move_base/make_plan";
while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
ROS_INFO("Waiting for service move_base/make_plan to become available");
}

ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
if (!serviceClient) {
ROS_FATAL("Could not initialize get plan service from %s",
serviceClient.getService().c_str());
return -1;
}
nav_msgs::GetPlan srv;

fillPathRequest(srv.request);
if (!serviceClient) {
ROS_FATAL("Persistent service connection to %s failed",
serviceClient.getService().c_str());
return -1;
}
ROS_INFO("conntect to %s",serviceClient.getService().c_str());
callPlanningService(serviceClient, srv);


}

