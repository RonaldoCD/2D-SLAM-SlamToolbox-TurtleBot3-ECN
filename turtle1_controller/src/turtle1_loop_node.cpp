#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleOneLine : public rclcpp::Node
{
public:
  TurtleOneLine()
  : Node("turtle_loop")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 1, std::bind(&TurtleOneLine::pose_callback, this, std::placeholders::_1));
  }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
     if(msg->x < 8.54445 && msg->x > 2.54445){
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 1.0;
        twist.angular.z = 0.0;
        publisher_->publish(twist);
      }else{
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 1.0;
        twist.angular.z = 0.5;
        publisher_->publish(twist);
      }

    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleOneLine>());
  rclcpp::shutdown();
  return 0;
}

/*
soso@ubuntuM2:~/ros2_ws$ ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

soso@ubuntuM2:~/ros2_ws$ ros2 topic info /turtle1/cmd_vel 
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1

soso@ubuntuM2:~/ros2_ws$ ros2 topic info /turtle1/pose 
Type: turtlesim/msg/Pose
Publisher count: 1
Subscription count: 0

soso@ubuntuM2:~/ros2_ws$ ros2 topic info /turtle1/color_sensor 
Type: turtlesim/msg/Color
Publisher count: 1
Subscription count: 0
*/