#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

class RealsenseClosestNode : public rclcpp::Node
{
public:
  RealsenseClosestNode()
  : Node("realsense_closest_node"),
    published_(false),
    best_depth_(std::numeric_limits<float>::max())
  {
    point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_leaf_point", 10);
    edge_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_leaf_edge_point", 10);

    cv::namedWindow("Result", cv::WINDOW_NORMAL);

    cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    profile_ = pipe_.start(cfg_);
    auto dev = profile_.get_device();
    auto depth_sensor = dev.first<rs2::depth_sensor>();
    depth_scale_ = depth_sensor.get_depth_scale();

    align_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);

    start_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&RealsenseClosestNode::processFrames, this));
  }

private:
  float depth_scale_;

  void processFrames()
  {
    if (published_) return;

    rs2::frameset frames;
    if (!pipe_.poll_for_frames(&frames)) return;
    auto aligned_frames = align_->process(frames);

    auto depth_frame = aligned_frames.get_depth_frame();
    auto color_frame = aligned_frames.get_color_frame();

    if (!depth_frame || !color_frame) return;

    cv::Mat depth_mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                      CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat color_mat(cv::Size(color_frame.get_width(), color_frame.get_height()),
                      CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

    cv::Mat depth_m;
    depth_mat.convertTo(depth_m, CV_32F, depth_scale_);

    // ★ 修正：深度マスク範囲を絞る（0.3〜0.8m）
    cv::Mat mask = (depth_m > 0.3) & (depth_m < 0.8);

    // デバッグ：マスク確認
    cv::imshow("Mask", mask * 255);

    cv::Mat filtered_depth = cv::Mat::zeros(depth_m.size(), CV_32F);
    depth_m.copyTo(filtered_depth, mask);

    // ★ 修正：normalize使用で安定化
    cv::Mat normalized_depth;
    cv::normalize(filtered_depth, normalized_depth, 0, 255, cv::NORM_MINMAX, CV_8U, mask);

    cv::Mat blurred;
    cv::GaussianBlur(normalized_depth, blurred, cv::Size(5,5), 0);

    cv::Mat thresh;
    cv::threshold(blurred, thresh, 1, 255, cv::THRESH_BINARY);

    // デバッグ：二値化画像確認
    cv::imshow("Thresh", thresh);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float best_depth_local = std::numeric_limits<float>::max();
    double best_area_local = 0.0;
    cv::Point best_center_local(-1,-1);
    std::vector<cv::Point> best_contour_local;
    cv::Point closest_point_local(-1,-1);

    for (const auto& contour : contours) {
      double area = cv::contourArea(contour);

      // ★ 修正：最小面積を増やす
      if (area < 1500) continue;

      cv::Moments M = cv::moments(contour);
      if (M.m00 == 0) continue;
      int cx = int(M.m10 / M.m00);
      int cy = int(M.m01 / M.m00);

      float depth_val = depth_m.at<float>(cy, cx);
      if (depth_val <= 0.0f || std::isnan(depth_val)) continue;

      double min_dist = std::numeric_limits<double>::max();
      cv::Point closest_point;
      for (const auto& pt : contour) {
        double dist = cv::norm(pt - cv::Point(cx, cy));
        if (dist < min_dist) {
          min_dist = dist;
          closest_point = pt;
        }
      }

      if (depth_val < best_depth_local || (std::abs(depth_val - best_depth_local) < 1e-6 && area > best_area_local)) {
        best_depth_local = depth_val;
        best_area_local = area;
        best_center_local = cv::Point(cx, cy);
        best_contour_local = contour;
        closest_point_local = closest_point;
      }
    }

    if (best_depth_local < best_depth_) {
      best_depth_ = best_depth_local;
      best_area_ = best_area_local;
      best_leaf_ = best_center_local;
      best_contour_ = best_contour_local;
      best_cut_ = closest_point_local;
      latest_color_ = color_mat.clone();
      latest_depth_mat_ = depth_mat.clone();
    }

    if ((this->now() - start_time_).seconds() > 10.0 && !published_ && best_depth_ < std::numeric_limits<float>::max()) {
      publishPointsAndShow();
      published_ = true;
    }

    cv::waitKey(1);  // デバッグ表示維持
  }

  void publishPointsAndShow()
  {
    rs2_intrinsics intr;
    auto stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    intr = stream.get_intrinsics();
    intr.model = RS2_DISTORTION_NONE;

    float center_pixel[2] = {(float)best_leaf_.x, (float)best_leaf_.y};
    float center_depth_m = best_depth_;

    float center_point3d[3];
    rs2_deproject_pixel_to_point(center_point3d, &intr, center_pixel, center_depth_m);

    if (best_leaf_.x < intr.ppx) center_point3d[0] = -std::abs(center_point3d[0]);
    else                         center_point3d[0] =  std::abs(center_point3d[0]);
    if (best_leaf_.y < intr.ppy) center_point3d[1] = -std::abs(center_point3d[1]);
    else                         center_point3d[1] =  std::abs(center_point3d[1]);

    geometry_msgs::msg::PointStamped center_msg;
    center_msg.header.stamp = this->now();
    center_msg.header.frame_id = "camera_depth_optical_frame";
    center_msg.point.x = center_point3d[0];
    center_msg.point.y = center_point3d[2];
    center_msg.point.z = -center_point3d[1];
    point_pub_->publish(center_msg);

    float edge_pixel[2] = {(float)best_cut_.x, (float)best_cut_.y};
    float edge_point3d[3];
    rs2_deproject_pixel_to_point(edge_point3d, &intr, edge_pixel, center_depth_m);

    if (best_cut_.x < intr.ppx) edge_point3d[0] = -std::abs(edge_point3d[0]);
    else                        edge_point3d[0] =  std::abs(edge_point3d[0]);
    if (best_cut_.y < intr.ppy) edge_point3d[1] = -std::abs(edge_point3d[1]);
    else                        edge_point3d[1] =  std::abs(edge_point3d[1]);

    geometry_msgs::msg::PointStamped edge_msg;
    edge_msg.header.stamp = this->now();
    edge_msg.header.frame_id = "camera_depth_optical_frame";
    edge_msg.point.x = edge_point3d[0];
    edge_msg.point.y = center_point3d[2];
    edge_msg.point.z = -edge_point3d[1];
    edge_point_pub_->publish(edge_msg);

    cv::Mat vis = latest_color_.clone();
    cv::drawContours(vis, std::vector<std::vector<cv::Point>>{best_contour_}, -1, cv::Scalar(0, 255, 0), 2);
    cv::circle(vis, best_leaf_, 5, cv::Scalar(0, 255, 255), -1);
    cv::circle(vis, best_cut_, 5, cv::Scalar(0, 0, 255), -1);

    cv::putText(vis, "Best depth: " + std::to_string(int(best_depth_ * 1000)) + " mm",
                best_leaf_ + cv::Point(10,0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);

    double offset_xy = std::sqrt(center_point3d[0]*center_point3d[0] + center_point3d[1]*center_point3d[1]);
    cv::putText(vis, "Offset from center XY: " + std::to_string(offset_xy) + " m",
                best_leaf_ + cv::Point(10,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);

    int img_center_x = vis.cols / 2;
    int img_center_y = vis.rows / 2;
    cv::line(vis, cv::Point(img_center_x, 0), cv::Point(img_center_x, vis.rows - 1), cv::Scalar(0, 0, 255), 1);
    cv::line(vis, cv::Point(0, img_center_y), cv::Point(vis.cols - 1, img_center_y), cv::Scalar(255, 0, 0), 1);

    cv::imshow("Result", vis);
    cv::waitKey(0);
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr edge_point_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rs2::pipeline pipe_;
  rs2::config cfg_;
  rs2::pipeline_profile profile_;
  std::unique_ptr<rs2::align> align_;

  rclcpp::Time start_time_;
  bool published_;
  float best_depth_;
  double best_area_;
  cv::Point best_leaf_;
  cv::Point best_cut_;
  std::vector<cv::Point> best_contour_;
  cv::Mat latest_color_;
  cv::Mat latest_depth_mat_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealsenseClosestNode>());
  rclcpp::shutdown();
  return 0;
}
