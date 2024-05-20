# include "rclcpp/rclcpp.hpp"
# include <chrono>
# include <iostream>
# include <fstream>
# include "nav_msgs/msg/path.hpp"
# include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
# include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Path;
using geometry_msgs::msg::PoseStamped;
using std::placeholders::_1;

class PathDrawerNode : public rclcpp::Node {
public:
    PathDrawerNode() : Node("path_drawer"){
        odom_ctr = 0;
        
        odom_subs = this->create_subscription<Odometry>("/waffle2/odom", 
                                                        10, 
                                                        std::bind(&PathDrawerNode::odom_subs_callback, this, _1));
        pose_subs = this->create_subscription<PoseWithCovarianceStamped>("/pose",
                                                        1, 
                                                        std::bind(&PathDrawerNode::pose_subs_callback, this, _1));
        
        estimated_path_pub = this->create_publisher<Path>("/estimated_path", 10);
        odom_path_pub = this->create_publisher<Path>("/odom_path", 10);

        timer_ = this->create_wall_timer(500ms, std::bind(&PathDrawerNode::publish_paths, this));
        
        RCLCPP_INFO(this->get_logger(), "Node initialized");
        est_path_msg.header.frame_id = "map";
        odom_path_msg.header.frame_id = "map";
    }
 
private:

    rclcpp::Subscription<Odometry>::SharedPtr odom_subs;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_subs;
    rclcpp::Publisher<Path>::SharedPtr estimated_path_pub;
    rclcpp::Publisher<Path>::SharedPtr odom_path_pub;
    
    Path est_path_msg = Path();
    Path odom_path_msg = Path();

    rclcpp::TimerBase::SharedPtr timer_;
    int odom_ctr;

    void pose_subs_callback(const PoseWithCovarianceStamped::SharedPtr msg)
    {
        PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = msg->pose.pose.position.x;
        pose_msg.pose.position.y = msg->pose.pose.position.y;
        pose_msg.pose.position.z = msg->pose.pose.position.z;
        est_path_msg.poses.push_back(pose_msg);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Images list size: " << images_list.size());
    }

    void odom_subs_callback(const Odometry::SharedPtr msg)
    {
        if (odom_ctr % 20 == 0){
            PoseStamped odom_pose_msg;
            odom_pose_msg.header.frame_id = "map";
            odom_pose_msg.pose.position.x = msg->pose.pose.position.x;
            odom_pose_msg.pose.position.y = msg->pose.pose.position.y;
            odom_pose_msg.pose.position.z = msg->pose.pose.position.z;
            odom_path_msg.poses.push_back(odom_pose_msg);
        }
        odom_ctr++;
        // int n = msg->poses.size();
        // RCLCPP_INFO_STREAM(this->get_logger(), "Poses size: " << n << msg->header.frame_id);        
    }

    void publish_paths()
    {
        estimated_path_pub->publish(est_path_msg);
        odom_path_pub->publish(odom_path_msg);
    }

};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<PathDrawerNode>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}