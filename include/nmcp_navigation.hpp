#include <nmcp.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

class NMPCControllerROS
{
  public:
    NMPCControllerROS();
    void run();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher control_pub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber loc_sub_;
    //ros::Subscriber initial_state_sub_;
    nmpc_controller::NMPCController nmpc_;
    bool is_goal_set;
    double publish_rate_;
    double roll_, pitch_, yaw_;

    std::vector<double> u_opt;
    std::vector<double> x_ref;
    std::vector<double> u_ref;
    std::vector<double> x0;

    void setGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void setCurrentState(const nav_msgs::Odometry::ConstPtr& msg);
    void setCurrentStateLoc(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void controlLoop();

};