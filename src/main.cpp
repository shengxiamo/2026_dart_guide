#include "io/camera.hpp"
#include "tasks/light_detect/Detect.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/yaml.hpp"
#include <cmath>
#include <chrono>
#include <iostream>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    std::string config_path = "configs/test.yaml";
    
    io::Camera camera(config_path);
    LightDetect light_detect(config_path);
    
    io::Gimbal gimbal(config_path);
    
    cv::Mat img;
    bool is_startup = true; 

    

    auto yaml = tools::load(config_path);
    auto camera_matrix = yaml["camera_matrix"].as<std::vector<double>>();
    const double fx = camera_matrix[0];
    const double cx = camera_matrix[2];

    while (true) {
        std::chrono::steady_clock::time_point timestamp;
        camera.read(img, timestamp);
        
        if (img.empty()) {
            std::cout << "Empty frame!" << std::endl;
            continue;
        }

        cv::namedWindow("Camera Image", cv::WINDOW_AUTOSIZE);
        
        auto lights = light_detect.detect(img, cv::Size2d(640, 480), 0, is_startup);
        is_startup = false; 

        cv::resize(img, img, cv::Size2d(640, 480));

        for (const auto &light : lights) {
            cv::rectangle(img, light.box, cv::Scalar(0, 255, 0), 2);
            cv::circle(img, light.center_point, 5, cv::Scalar(255, 0, 0), -1);

            // 3.1 计算水平像素偏移（目标中心 - 图像中心）
            // float pixel_offset = light.center_point.x * 2.25 - cx;
            // float angle_offset_rad = std::atan2(pixel_offset, fx);
            float pixel_offset = light.center_point.x - (640 / 2.0f);
            // 3.4 构造发送结构体并发送
            io::VisionToGimbal send_data;
            send_data.yaw_offset = pixel_offset;

            printf("Pixel Offset: %.2f\n", pixel_offset);
            gimbal.send(send_data);
        // 原有：显示图像
        cv::imshow("Camera Image", img);
        if (cv::waitKey(1) == 'q') break;
    }
    }

    cv::destroyAllWindows();
    return 0;
}
