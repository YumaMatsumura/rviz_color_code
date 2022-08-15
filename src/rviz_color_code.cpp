#include "rviz_color_code/rviz_color_code.hpp"

RvizColorCode::RvizColorCode(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("color_ball", name_space, options)
{
  using namespace std::chrono_literals;
  
  RCLCPP_INFO(this->get_logger(), "Creating");
  
  sphere_radius_   = this->declare_parameter<double>("sphere_radius", 0.2);
  space_length_    = this->declare_parameter<double>("space_length", 0.2);
  column_size_      = this->declare_parameter<int>("column_size", 41);
  division_number_ = this->declare_parameter<int>("division_number", 12);
  
  rclcpp::QoS qos(1);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/color_balls", qos);
  timer_       = this->create_wall_timer(
    400ms, std::bind(&RvizColorCode::publishMarkerArray, this)
  );
}

RvizColorCode::~RvizColorCode()
{
  RCLCPP_INFO(this->get_logger(), "Destroying");
}

void RvizColorCode::publishMarkerArray()
{
  rclcpp::Time now = this->get_clock()->now();
  
  auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  marker_array->markers.resize(division_number_ * division_number_ * division_number_ * 2);
  
  const double marker_to_marker_length = 2 * sphere_radius_ + space_length_;
  const int last_marker_number = division_number_ * division_number_ * division_number_ - 1;
  const int row_size_ = std::floor(last_marker_number/column_size_) + 1;
  
  for(int i = 0; i < division_number_; i++)
  {
    for(int j = 0; j < division_number_; j++)
    {
      for(int k = 0; k < division_number_; k++)
      {
        const int marker_number = division_number_ * division_number_ * i + division_number_ * j + k;
        const double red   = static_cast<double>(i) / (division_number_ - 1.0);
        const double green = static_cast<double>(j) / (division_number_ - 1.0);
        const double blue  = static_cast<double>(k) / (division_number_ - 1.0);
        const double pos_x = 
          (static_cast<double>(std::floor(marker_number/column_size_)) - static_cast<double>(row_size_-1)/2.0)
          * marker_to_marker_length;
        const double pos_y = 
          (static_cast<double>(marker_number%column_size_) - static_cast<double>(column_size_-1)/2.0)
          * marker_to_marker_length;
          
        visualization_msgs::msg::Marker marker_ball;
        marker_ball.header.frame_id = "map";
        marker_ball.header.stamp = now;
        marker_ball.id = 2 * marker_number;
        marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
        marker_ball.action = visualization_msgs::msg::Marker::ADD;
        marker_ball.pose.position.x = pos_x;
        marker_ball.pose.position.y = pos_y;
        marker_ball.pose.position.z = 0.5;
        marker_ball.pose.orientation.w = 1.0;
        marker_ball.scale.x = sphere_radius_ * 2;
        marker_ball.scale.y = sphere_radius_ * 2;
        marker_ball.scale.z = sphere_radius_ * 2;
        marker_ball.color.r = red;
        marker_ball.color.g = green;
        marker_ball.color.b = blue;
        marker_ball.color.a = 1.0;
        marker_array->markers.push_back(marker_ball);
        
        visualization_msgs::msg::Marker marker_text;
        marker_text.header.frame_id = "map";
        marker_text.header.stamp = now;
        marker_text.id = 2 * marker_number + 1;
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;
        marker_text.pose.position.x = pos_x;
        marker_text.pose.position.y = pos_y;
        marker_text.pose.position.z = 1.0;
        marker_text.pose.orientation.w = 1.0;
        marker_text.scale.z = 0.3;
        marker_text.color.r = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.b = 1.0;
        marker_text.color.a = 1.0;
        marker_text.text = std::to_string(marker_number);
        marker_array->markers.push_back(marker_text);
      }
    }
  }
  
  pub_markers_->publish(std::move(marker_array));
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<RvizColorCode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}

