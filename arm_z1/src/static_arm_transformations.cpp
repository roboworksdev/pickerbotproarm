#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticTFBroadcasterNode : public rclcpp::Node
{
public:
  StaticTFBroadcasterNode() : Node("static_tf_broadcaster_node")
  {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.frame_id = "arm_base_link";
    static_transform.child_frame_id = "lower_position";
    static_transform.transform.translation.x = 0.30;  
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.10;

    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_transform);


  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTFBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
