#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class RvizColorCode : public rclcpp::Node
{
public:
  RvizColorCode(
    const std::string& name_space = "",
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  ~RvizColorCode();

private:
  void publishMarkerArray();
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double sphere_radius_;
  double space_length_;
  int column_size_;
  int division_number_;
};

