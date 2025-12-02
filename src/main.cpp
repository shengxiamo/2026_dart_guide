#include <fmt/core.h>

#include <chrono>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"

#include "tasks/light_detect/Detect.hpp"
#include <opencv2/opencv.hpp>

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/test.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>(0);
    if (cli.has("help") || config_path.empty()) {
      cli.printMessage();
      return 0;
    }

    io::Gimbal gimbal(config_path);
    io::Camera camera(config_path);
    LightDetect light_detect(config_path);

    cv::Mat img_curr, img_prev; // 定义当前帧和上一帧
    std::chrono::steady_clock::time_point timestamp;
    bool is_startup = true;

    while (true) {
        // 1. 读取当前帧
        camera.read(img_curr, timestamp);
        
        if (img_curr.empty()) continue;

        // 2. 执行推理
        // 注意：detect 内部会启动 img_curr 的推理，并返回 img_prev 的结果
        auto lights = light_detect.detect(img_curr, cv::Size2d(640, 480), 0, is_startup);

        // 3. 处理显示逻辑
        if (!is_startup) {
            // 此时 lights 对应的是 img_prev，所以我们在 img_prev 上画图
            auto dt = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - std::chrono::steady_clock::now()).count();
            fmt::print("Frame time: {} ms, FPS: {:.1f}\n", dt, 1000.0 / dt);
            if (!img_prev.empty()) {
                // 为了显示美观，可以在这里 resize 用于显示的图，而不是影响推理输入
                cv::Mat img_show = img_prev.clone(); 
                cv::resize(img_show, img_show, cv::Size2d(640, 480));

                // 取置信度最大的目标进行显示
                std::sort(lights.begin(), lights.end(),
                          [](const OpenvinoInfer::Light &a, const OpenvinoInfer::Light &b) {
                              return a.score > b.score;
                          });
                if (!lights.empty()) {
                    const auto &light = lights[0];
                    cv::rectangle(img_show, light.box, cv::Scalar(0, 255, 0), 2);
                    cv::circle(img_show, light.center_point, 5, cv::Scalar(255, 0, 0), -1);

                    // 串口发送
                    gimbal.send(light.center_point.x - img_show.cols / 2); // 发送偏移量给云台
                }

                
                cv::imshow("Camera Image", img_show);
            }
        }

        // 4. 更新状态
        // 将当前帧存为“上一帧”，供下一次循环显示使用
        // 注意：必须使用 clone()，因为相机驱动可能会复用内存
        img_prev = img_curr.clone(); 
        is_startup = false;

        if (cv::waitKey(1) == 'q') break;
    }

    cv::destroyAllWindows();
    return 0;
}