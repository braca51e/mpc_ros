#include <nmcp_navigation.hpp>

NMPCControllerROS::NMPCControllerROS() : nh_(), nh_private_("~")
//NMPCControllerROS::NMPCControllerROS()
{
  nh_private_.param<double>("publish_rate", publish_rate_, 60.0);


  nh_private_ = ros::NodeHandle("~");
  //nh_ = ros::NodeHandle();
  control_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  //amcl_pose_sub_ = nh_.subscribe("/amcl_pose", 1, &NMPCControllerROS::setCurrentState, this);
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &NMPCControllerROS::setGoal, this);
  //odom_sub_ = nh_.subscribe("/odom", 1, &NMPCControllerROS::setCurrentState, this);
  loc_sub_ = nh_.subscribe("/amcl_pose", 1, &NMPCControllerROS::setCurrentStateLoc, this);
  
  is_goal_set = false;
  nmpc_.setUp();
  x_ref = {0.0, 0.0, 0.0};
  u_ref = {0.0, 0.0};
  nmpc_.setReference(x_ref, u_ref);

}

void NMPCControllerROS::setGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // tf2 transform from map to base_footprint
  //geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform("base_footprint", "map", ros::Time(0));
  //geometry_msgs::PointStamped point_in_base_footprint;
  //tf2::doTransform(msg, point_in_base_footprint, transform_stamped);

  // convert quaternion to roll pitch yaw
  tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  //tf::Quaternion q(point_in_base_footprint.pose.orientation.x, point_in_base_footprint.pose.orientation.y, point_in_base_footprint.pose.orientation.z, point_in_base_footprint.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // ROS info goal
  ROS_INFO("Goal: x: %f, y: %f, theta: %f", msg->pose.position.x, msg->pose.position.y, yaw_);
  x_ref = {msg->pose.position.x, msg->pose.position.y, yaw_};
  u_ref = {0.0, 0.0};
  nmpc_.setReference(x_ref, u_ref);
  is_goal_set = true;
}

void NMPCControllerROS::controlLoop()
{
    // @TODO: Get most rescen state from odom
    u_opt = nmpc_.solve(x0);

    //ROS_INFO("Control opt: v: %f, w: %f", u_opt[0], u_opt[1]);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = u_opt[0];
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.z = u_opt[1];
    control_pub_.publish(cmd_vel);
}

void NMPCControllerROS::run()
{
  ros::Rate rate(publish_rate_);

  while (ros::ok())
  {
    ros::spinOnce();

    // check closenees to goal
    if (is_goal_set && !x0.empty())
    {
      // run controol loop
      double dist = sqrt(pow(x0[0] - x_ref[0], 2) + pow(x0[1] - x_ref[1], 2) +  pow(x0[2] - x_ref[2], 2)) ;
      if (dist < 0.1)
      {
        ROS_INFO("Goal reached!");
        is_goal_set = false;
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.z = 0.0;
        control_pub_.publish(cmd_vel);
      }
      else{
        //ROS_INFO("Distance to goal: %f", dist);
        controlLoop();
        }
        
    }

    rate.sleep();
  }
}

void NMPCControllerROS::setCurrentState(const nav_msgs::Odometry::ConstPtr& msg){


  // get orientation from quaternion
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // set state
  x0 = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_};

  // ros info current state
  //ROS_INFO("Current state: x: %f, y: %f, theta: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_);
}

void NMPCControllerROS::setCurrentStateLoc(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{


  // get orientation from quaternion
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // set state
  x0 = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_};

  // ros info current state
  //ROS_INFO("Current state: x: %f, y: %f, theta: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_);
}
