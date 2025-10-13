#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rsutil.h>

class RealsenseClosestFromRGBD : public rclcpp::Node
{
public:
    RealsenseClosestFromRGBD()
    : Node("realsense_closest_from_rgbd"),
      published_(false),
      best_depth_(std::numeric_limits<float>::max()),
      depth_scale_(0.001f)
    {
        point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_leaf_point", 10);
        edge_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_leaf_edge_point", 10);

        sub_rgbd_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
            "/devices/ee_camera/realsense_node/rgbd", 10,
            std::bind(&RealsenseClosestFromRGBD::rgbdCallback, this, std::placeholders::_1)
        );

        sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/devices/ee_camera/realsense_node/depth/camera_info", 10,
            std::bind(&RealsenseClosestFromRGBD::infoCallback, this, std::placeholders::_1)
        );

        cv::namedWindow("Processing", cv::WINDOW_NORMAL);


        start_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&RealsenseClosestFromRGBD::tryProcess, this)
        );
    }

private:
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr sub_rgbd_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr edge_point_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat color_img_, depth_img_;
    bool rgbd_ready_ = false;
    rs2_intrinsics intr_;
    bool published_;
    float depth_scale_;
    float best_depth_;
    double best_area_;
    cv::Point best_leaf_;
    cv::Point best_cut_;
    std::vector<cv::Point> best_contour_;
    cv::Mat latest_color_;
    rclcpp::Time start_time_;

    void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        intr_.width = msg->width;
        intr_.height = msg->height;
        intr_.ppx = msg->k[2];
        intr_.ppy = msg->k[5];
        intr_.fx  = msg->k[0];
        intr_.fy  = msg->k[4];
        intr_.model = RS2_DISTORTION_NONE;
        for (int i = 0; i < 5; ++i) intr_.coeffs[i] = 0;
    }

    void rgbdCallback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg)
    {
        try {
            color_img_ = cv_bridge::toCvCopy(msg->rgb, "bgr8")->image;
            if (msg->depth.encoding == "16UC1")
                depth_img_ = cv_bridge::toCvCopy(msg->depth, "16UC1")->image * depth_scale_;
            else
                depth_img_ = cv_bridge::toCvCopy(msg->depth, "32FC1")->image;

            rgbd_ready_ = true;
            tryProcess();
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void tryProcess()
    {
        if (!rgbd_ready_ || published_) return;

        cv::Mat depth_m = depth_img_;
        cv::Mat color_m = color_img_;

        // --- 深度マスク処理 ---
        // cv::Mat mask = (depth_m > 0.2) & (depth_m < 0.8);
        cv::Mat mask = (depth_m > 2.0) & (depth_m < 8.0);
        cv::Mat filtered_depth = cv::Mat::zeros(depth_m.size(), CV_32F);
        depth_m.copyTo(filtered_depth, mask);

        cv::Mat normalized_depth;
        cv::normalize(filtered_depth, normalized_depth, 0, 255, cv::NORM_MINMAX, CV_8U, mask);
        cv::Mat blurred;
        cv::GaussianBlur(normalized_depth, blurred, cv::Size(5,5), 0);

        cv::Mat thresh;
        cv::threshold(blurred, thresh, 1, 255, cv::THRESH_BINARY);

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
            if (area < 4000){
            RCLCPP_DEBUG(this->get_logger(), "Contour skipped (small area): %.1f", area);
            continue;
            }

            cv::Moments M = cv::moments(contour);
            if (M.m00 == 0){
                RCLCPP_DEBUG(this->get_logger(), "Contour skipped (invalid moments): %.3f", M.m00);
                continue;
            }
            int cx = int(M.m10 / M.m00);
            int cy = int(M.m01 / M.m00);

            int image_center_x = color_m.cols / 2;
            int image_center_y = color_m.rows / 2;
            int center_margin = 100;
            if (std::abs(cx - image_center_x) > center_margin || std::abs(cy - image_center_y) > center_margin) {
                RCLCPP_DEBUG(this->get_logger(), "Contour skipped (invalid center_magin):");
                continue;
            }

            float depth_val = depth_m.at<float>(cy, cx);
            if (depth_val <= 0.0f || std::isnan(depth_val)){
                RCLCPP_DEBUG(this->get_logger(), "Contour skipped (invalid depth_val):");
                continue;
            }

            double min_dist = std::numeric_limits<double>::max();
            cv::Point closest_point;
            for (const auto& pt : contour) {
                double dist = cv::norm(pt - cv::Point(cx, cy));
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_point = pt;
                }
            }

            if (depth_val < best_depth_local ||
               (std::abs(depth_val - best_depth_local) < 1e-6 && area > best_area_local)) {
                best_depth_local = depth_val;
                best_area_local = area;
                best_center_local = cv::Point(cx, cy);
                best_contour_local = contour;
                closest_point_local = closest_point;
                RCLCPP_DEBUG(this->get_logger(), "Contour skipped (NON SKIP):");
            }

        }

        if (best_depth_local < best_depth_) {
            best_depth_ = best_depth_local;
            best_area_ = best_area_local;
            best_leaf_ = best_center_local;
            best_contour_ = best_contour_local;
            best_cut_ = closest_point_local;
            latest_color_ = color_m.clone();
        }

        if ((this->now() - start_time_).seconds() > 10.0 && !published_ && best_depth_ < std::numeric_limits<float>::max()) {
            publishPointsAndShow();
            published_ = true;
            
        }

        // double minVal, maxVal;
        // cv::minMaxLoc(depth_m, &minVal, &maxVal);
        // RCLCPP_INFO(this->get_logger(), "depth_m min=%.3f max=%.3f", minVal, maxVal);
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "DEBUG INFO: sec, published=%s, best_depth=%.3f, max_depth=%.3e,b < m = %s",
        //     published_ ? "true" : "false",
        //     best_depth_,
        //     std::numeric_limits<float>::max(),
        //     best_depth_ < std::numeric_limits<float>::max() ? "true" : "false"
        // );
        // RCLCPP_INFO(this->get_logger(), "Contours found: %zu", contours.size());


        // --- 可視化 ---
        cv::Mat vis_frame = color_m.clone();
        for (const auto& contour : contours)
            cv::drawContours(vis_frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0,255,0), 1);
        cv::circle(vis_frame, best_leaf_, 4, cv::Scalar(0,255,255), -1);
        cv::imshow("Processing", vis_frame);
        cv::waitKey(1);

    }

    void publishPointsAndShow()
    {
        float center_pixel[2] = {(float)best_leaf_.x, (float)best_leaf_.y};
        float center_depth_m = best_depth_;

        float center_point3d[3];
        rs2_deproject_pixel_to_point(center_point3d, &intr_, center_pixel, center_depth_m);

        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_color_optical_frame";
        msg.point.x = center_point3d[0];
        msg.point.y = center_point3d[1];
        msg.point.z = center_point3d[2];
        point_pub_->publish(msg);

        float edge_pixel[2] = {(float)best_cut_.x, (float)best_cut_.y};
        float edge_point3d[3];
        rs2_deproject_pixel_to_point(edge_point3d, &intr_, edge_pixel, center_depth_m);

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

        cv::imshow("Result", vis);
        cv::waitKey(0);
    }
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<RealsenseClosestFromRGBD>());
//   rclcpp::shutdown();
//   return 0;
// }

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealsenseClosestFromRGBD>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
