#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem> // C++17 引入的文件系统库，用于处理文件和目录

namespace fs = std::filesystem; // 为了方便，给文件系统库起个别名

int main(int argc, char * argv[]) try {
    // 创建管道
    rs2::pipeline pipe;
    // 配置管道以使用默认设置（这里可以调整分辨率、格式和帧率）
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // 开始管道（打开相机）
    pipe.start(cfg);

    // 获取当前工作目录，并构建photo_dir路径
    std::string current_path = fs::current_path(); // 获取当前工作目录
    std::string photo_dir = current_path + "/picture"; // 构建photo_dir路径

    // 如果picture文件夹不存在，则创建它
    if (!fs::exists(fs::path(photo_dir))) {
        fs::create_directory(fs::path(photo_dir));
    }

    // 捕获一帧图像
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::frame color_frame = frames.get_color_frame();


    // 如果成功获取到彩色帧
    if (color_frame) {
        // 将图像转换为OpenCV的Mat格式
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // 构建文件名（使用当前时间的毫秒数来保证唯一性）
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto in_millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::string filename = photo_dir + "/photo_" + std::to_string(in_time_t) + "_" + std::to_string(in_millis.count()) + ".png";

        // 保存图像到文件
        cv::imwrite(filename, color_image);

        std::cout << "Image saved to " << filename << std::endl;
    } else {
        std::cerr << "Unable to retrieve color frame" << std::endl;
    }

    // 停止管道（关闭相机）
    pipe.stop();

    return 0;
} catch (const rs2::error & e) {
    std::cerr << "RealSense error: " << e.what() << std::endl;
    return -1;
} catch (const std::exception & e) {
    std::cerr << "Standard exception: " << e.what() << std::endl;
    return -1;
}