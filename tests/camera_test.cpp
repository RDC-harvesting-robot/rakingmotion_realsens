#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

cv::Point clicked_pixel(-1, -1);
cv::Mat latest_depth_m;

// マウスコールバック
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if(event == cv::EVENT_LBUTTONDOWN){
        clicked_pixel = cv::Point(x, y);
        std::cout << "Clicked pixel: (" << x << ", " << y << ")" << std::endl;
    }
}

int main()
{
    try {
        rs2::pipeline pipe;
        rs2::config cfg;

        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        rs2::pipeline_profile profile = pipe.start(cfg);
        auto dev = profile.get_device();
        auto depth_sensor = dev.first<rs2::depth_sensor>();
        float depth_scale = depth_sensor.get_depth_scale();

        rs2::align align_to_color(RS2_STREAM_COLOR);

        cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback("Color", onMouse, nullptr);

        while(true){
            rs2::frameset frames;
            if(!pipe.poll_for_frames(&frames)) continue;

            rs2::frameset aligned_frames = align_to_color.process(frames);
            rs2::video_frame color_frame = aligned_frames.get_color_frame();
            rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();

            int w = color_frame.get_width();
            int h = color_frame.get_height();

            cv::Mat color(cv::Size(w,h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth(cv::Size(w,h), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            depth.convertTo(latest_depth_m, CV_32F, depth_scale);

            cv::imshow("Color", color);
            if(cv::waitKey(1) == 27) break; // ESCで終了

            // クリックした場合のみ処理
            if(clicked_pixel.x >= 0 && clicked_pixel.y >= 0){
                int u = clicked_pixel.x;
                int v = clicked_pixel.y;

                int radius = 3; // 周辺探索 ±3ピクセル
                float min_depth = std::numeric_limits<float>::max();
                int min_u = u, min_v = v;

                for(int y = std::max(0,v-radius); y <= std::min(latest_depth_m.rows-1,v+radius); y++){
                    for(int x = std::max(0,u-radius); x <= std::min(latest_depth_m.cols-1,u+radius); x++){
                        float d = latest_depth_m.at<float>(y,x);
                        if(d>0 && d<min_depth){
                            min_depth = d;
                            min_u = x;
                            min_v = y;
                        }
                    }
                }

                if(min_depth == std::numeric_limits<float>::max()){
                    std::cout << "有効なDepthが見つかりません" << std::endl;
                } else {
                    auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
                    rs2_intrinsics intr = stream.get_intrinsics();

                    float pixel[2] = {(float)min_u, (float)min_v};
                    float point[3];
                    rs2_deproject_pixel_to_point(point, &intr, pixel, min_depth);

                    std::cout << "3D coordinates: X=" << point[0]
                              << " Y=" << point[1]
                              << " Z=" << point[2] << " (meters)" << std::endl;
                }

                clicked_pixel = cv::Point(-1,-1); // 一度表示したらリセット
            }
        }

    } catch(const rs2::error & e){
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch(const std::exception & e){
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
