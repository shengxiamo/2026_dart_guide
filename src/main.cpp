#include "io/camera.hpp"

#include "tasks/light_detect/Detect.hpp"
#include <opencv2/opencv.hpp>
int main() {
    io::Camera camera("/home/yagami_light/Desktop/2026_dart_guide/configs/test.yaml");
    
    // 修改 1: 路径改为 test.yaml
    // 修改 2: 将初始化移到 while 循环外部，避免每一帧都重新加载模型
    std::string config_path = "/home/yagami_light/Desktop/2026_dart_guide/configs/test.yaml";
    LightDetect light_detect(config_path);

    cv::Mat img;
    bool is_startup = true; // 1. 定义标志位

    while (true) {
        std::chrono::steady_clock::time_point timestamp;
        camera.read(img, timestamp);
        
        if (img.empty()) {
            std::cout << "Empty frame!" << std::endl;
            continue;
        }

        cv::namedWindow("Camera Image", cv::WINDOW_AUTOSIZE);
        
        // 2. 使用标志位，并在第一帧后将其置为 false
        auto lights = light_detect.detect(img, cv::Size2d(640, 480), 0, is_startup);
        is_startup = false; 

        cv::resize(img, img, cv::Size2d(640, 480));

        for (const auto &light : lights) {
            cv::rectangle(img, light.box, cv::Scalar(0, 255, 0), 2);
            cv::circle(img, light.center_point, 5, cv::Scalar(255, 0, 0), -1);
        }
        cv::imshow("Camera Image", img);
        if (cv::waitKey(1) == 'q') break;
    }

    cv::destroyAllWindows();
    return 0;
}