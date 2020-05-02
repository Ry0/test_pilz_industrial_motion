#include <iostream>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <motoman_viz_msgs/EuclideanLinkTrajectory.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class FKsolver
{
public:
  FKsolver(ros::NodeHandle &nh);

private:
  void DisplayTrajectoryCB(const sensor_msgs::JointState display_traj);
  void Hsv2Rgb(double hsv[], double rgb[]);

  ros::Publisher marker_pub;
  ros::Subscriber display_traj_sub;
  std::vector<visualization_msgs::Marker> marker_que;
  std::string target_frame;
  std::string model_group_name;
  int32_t count;
};

FKsolver::FKsolver(ros::NodeHandle &nh) : target_frame("link_t"),
                                          model_group_name("arm")
{
  ros::NodeHandle n("~");

  n.param("motoman_viz/target_frame", target_frame, target_frame);
  n.param("motoman_viz/model_group_name", model_group_name, model_group_name);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("joiint_state_maker", 1);
  display_traj_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &FKsolver::DisplayTrajectoryCB, this);
  count = 0;
  ROS_INFO("Motoman Viz : Initialized");
}

void FKsolver::DisplayTrajectoryCB(const sensor_msgs::JointState display_traj)
{
  ROS_INFO("joint states subscribed!!");

  tf::TransformListener listener;
  geometry_msgs::PoseStamped target_pose;

  geometry_msgs::PoseStamped source_pose;
  source_pose.header.frame_id = target_frame;
  source_pose.header.stamp = ros::Time(0);
  source_pose.pose.orientation.w = 1.0;

  std::string base_frame = "/world";

  try
  {
    ros::Time now = ros::Time::now();
    listener.waitForTransform(source_pose.header.frame_id, base_frame, ros::Time(0), ros::Duration(1.0));
    listener.transformPose(base_frame, source_pose, target_pose);
    ROS_INFO("x:%+5.2f, y:%+5.2f,z:%+5.2f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = count;
    count++;
    double hsv[3] = {double((count * 10) % 360), 1, 1};
    double rgb[3];
    Hsv2Rgb(hsv, rgb);
    ROS_INFO("r:%+5.2f, g:%+5.2f,b:%+5.2f", rgb[0], rgb[1], rgb[2]);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.pose.position.x = target_pose.pose.position.x;
    marker.pose.position.y = target_pose.pose.position.y;
    marker.pose.position.z = target_pose.pose.position.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = rgb[0];
    marker.color.g = rgb[1];
    marker.color.b = rgb[2];
    marker.color.a = 1.0f;
    marker_que.push_back(marker);
    if (marker_que.size() > 10000)
      marker_que.erase(marker_que.begin());

    marker_array.markers = marker_que;
    marker_pub.publish(marker_array);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void FKsolver::Hsv2Rgb(double hsv[], double rgb[])
{
  int Hi;
  float f;
  float p;
  float q;
  float t;

  Hi = fmod(floor(hsv[0] / 60.0f), 6.0f);
  f = hsv[0] / 60.0f - Hi;
  p = hsv[2] * (1.0f - hsv[1]);
  q = hsv[2] * (1.0f - f * hsv[1]);
  t = hsv[2] * (1.0f - (1.0f - f) * hsv[1]);

  if (Hi == 0)
  {
    rgb[0] = hsv[2];
    rgb[1] = t;
    rgb[2] = p;
  }
  if (Hi == 1)
  {
    rgb[0] = q;
    rgb[1] = hsv[2];
    rgb[2] = p;
  }
  if (Hi == 2)
  {
    rgb[0] = p;
    rgb[1] = hsv[2];
    rgb[2] = t;
  }
  if (Hi == 3)
  {
    rgb[0] = p;
    rgb[1] = q;
    rgb[2] = hsv[2];
  }
  if (Hi == 4)
  {
    rgb[0] = t;
    rgb[1] = p;
    rgb[2] = hsv[2];
  }
  if (Hi == 5)
  {
    rgb[0] = hsv[2];
    rgb[1] = p;
    rgb[2] = q;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forward_kinematics_solver_joint_state_node");
  ros::NodeHandle n;
  FKsolver fk_solver(n);

  float HZ = 100;
  if (HZ > 0)
  {
    ros::Rate loop_rate(HZ);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  else
    ros::spin();

  return 0;
}
