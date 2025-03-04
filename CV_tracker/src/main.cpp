#include "tasks/detector.hpp"
#include "tools/img_tools.hpp"
#include "tasks/kalmanfilter.hpp"
#include "tasks/serial.hpp"
#include "fmt/core.h"
#include <cmath>
#include <iostream>
#include <cstring>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

const double gammavalue = 1.20;
const float freshrate = 0.0133f;
const float exposure = 100.0f; 

// 传输给电控的结构体
struct armorPlate {
    float tvec_y = 1.0f;
    float tvec_z = 1.0f;
    float is_detected = 0.0f;
    float armor_length = 0.0f;
    float armor_width = 0.0f;
    float orientation = 0.0f;
};

static armorPlate plate;

// clang-format off
//  相机内参
static const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) <<  590.8022 , 0                  , 333.2000, 
                                0                 , 588.6825 , 247.5028 , 
                                0                 , 0              , 1);
// 畸变系数
static const cv::Mat distort_coeffs =
   // (cv::Mat_<double>(1, 5) << 0.0511, 0.3741, 0.0888, 0.0016, 0.0026);
   (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
// clang-format on

static const double LIGHTBAR_LENGTH = 0.056; // 灯条长度    单位：米
static const double ARMOR_WIDTH = 0.135;     // 装甲板宽度  单位：米


// #### Task 01 ############################################
// object_points 是 物体局部坐标系下 n个点 的坐标。a
// 对于我们而言，也就是装甲板坐标系下4个点的坐标。
// 请你填写下面的 object_points:

static const std::vector<cv::Point3f> object_points {
    {-ARMOR_WIDTH / 2  , -LIGHTBAR_LENGTH / 2 , 0 },  // 点 1
    {ARMOR_WIDTH / 2   , -LIGHTBAR_LENGTH / 2 , 0 },  // 点 2
    {ARMOR_WIDTH / 2   , LIGHTBAR_LENGTH  / 2 , 0 },  // 点 3
    {-ARMOR_WIDTH / 2   , LIGHTBAR_LENGTH  / 2 , 0 }   // 点 4
};

// 提示：
// - 装甲板坐标系是三维的坐标系，但是四个点都在 z 坐标为 0 的平面上，所以已经为你填写了四个 0 。
// - 在上方定义有 灯条长度 和 装甲板宽度，你应当用 "± ARMOR_WIDTH / 2" 这样的写法来填写。
// #########################################################
void adjustGamma(cv::Mat& img, double gamma) {
    cv::Mat lookupTable(1, 256, CV_8U);
    uchar* p = lookupTable.ptr();
    for (int i = 0; i < 256; i++) {
        p[i] = cv::saturate_cast<uchar>(pow((i/255.0), gamma) * 255.0);
    }
    cv::LUT(img, lookupTable, img);
}

int main(int argc, char *argv[])
{
    Serial serial("/dev/ttyUSB0", B115200); //更改串口名
    auto_aim::Detector detector;

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 0);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);
    

    rs2::device dev = pipe.get_active_profile().get_device();
    
    // get the color sensor
    auto sensor_list = dev.query_sensors();
    for(auto& sensor : sensor_list) {
        if (sensor.supports(RS2_OPTION_EXPOSURE)) {
            // disable auto-exposure
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);

            float exposure_value = exposure;
            sensor.set_option(RS2_OPTION_EXPOSURE, exposure_value);
            //std::cout << "Exposure set to:" << exposure_value << "microseconds" << std::endl;
        }
    }

    kalman_filter kf;

    //cv::Mat img;
    while (true)
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames();
        // Get a frame from the frameset
        rs2::frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();

        if(!color_frame) {
            std::cerr << "couldn't receive color frame" << std::endl;
        }

        // Conver frame to OpenCv's Mat format
        const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color_frame.get_data());
        cv::Mat img(cv::Size(640, 480), CV_8UC3, (void*)color_data, cv::Mat::AUTO_STEP);
        
        // Get the profile of the color stream
        const rs2::stream_profile& color_profile = color_frame.get_profile();
        // Get the video stream profile and query the framerate
        const rs2::video_stream_profile& video_profile = color_profile.as<rs2::video_stream_profile>();
        float framerate = video_profile.fps();


        adjustGamma(img, gammavalue);
        auto armors = detector.detect(img);
        const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth.get_data());

        int width = depth.get_width();
        int height = depth.get_height();
        uint16_t min_distance = UINT16_MAX;
        int min_x = 0, min_y = 0;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                uint16_t cur_distance = depth_data[y * width + x];
                if (cur_distance < min_distance && cur_distance > 0) { // 通常，0 表示无效深度值
                    min_distance = cur_distance;
                    min_x = x;
                    min_y = y;
                }
            }
        }
       
        float angle = float(min_x) / 640.0f * 87.0f;
        // std::cout << angle << std::endl;
        
        // if(min_distance <= 400) {
        //     if(angle > 0 && angle < 43.5) {
        //         angle =  angle;
        //     } else if(angle >= 43.5 && angle < 87) {
        //         angle =  angle - 87;
        //     }
        // } else {
        //     angle = 0;
        // }

        if(min_distance <= 400) {
            angle = 1;
        } else {
            angle = 0;
        }

        std::cout << "min_distance: " << min_distance << " x: " << min_x << " y; " << min_y << std::endl;
        std::cout << " angle to turn: " << angle << std::endl;

        if (!armors.empty())
        {
            auto armor = armors.front();           // 如果识别到了大于等于一个装甲板，则取出第一个装甲板来处理
            tools::draw_points(img, armor.points); // 绘制装甲板
            
            std::vector<float> sorted_x; 
            std::vector<float> sorted_y; 

            for(size_t i = 0; i < armor.points.size(); i++) {
                sorted_x.push_back(armor.points[i].x);
                sorted_y.push_back(armor.points[i].y);
            }
 
            // // 使用 std::sort 对 numbers 进行排序
            std::sort(sorted_x.begin(), sorted_x.end());
            std::sort(sorted_y.begin(), sorted_y.end());

            float roi_length = sorted_x.back() - sorted_x.front();
            float roi_width = sorted_y.back() - sorted_y.front();

            // #### Task 02 ############################################
            // img_points 是 像素坐标系下 n个点 的坐标。
            // 对于我们而言，也就是 照片上 装甲板 4个点的坐标。
            // 请你填写下面的 img_points:
            //
            std::vector<cv::Point2f> img_points{ 
                armor.left.top, 
                armor.right.top,
                armor.right.bottom,
                armor.left.bottom};
            //
            // 提示：
            // - 看看 Armor 结构体有哪些成员。
            // - armor 的成员 left 和 right 是两根灯条(Lightbar)。
            // - 灯条Lightbar也是结构体，灯条的顶部和底部端点就是我们要的点了。
            // #########################################################



            // #### Task 03 ############################################
            cv::Mat rvec, tvec;
            // 所有要传入的值都已经具备了。现在调用 solvePnP 解算装甲板位姿，
            // rvec 和 tvec 用于存储 solvePnP 输出的结果。
            // 你需要在下面填写 输入给 solvePnP 的参数：
            //
            cv::solvePnP(object_points, img_points, camera_matrix, distort_coeffs, rvec, tvec);
            double armor_x = tvec.at<double>(0);
            double armor_y = tvec.at<double>(1);
            double armor_z = tvec.at<double>(2);

            //
            // #########################################################



            // #### Task 04 ############################################
            // 现在，draw_text 只打印 0.0
            // 请你改写下面draw_text的参数，把解得的 tvec 和 rvec 打印出来
            //
            //tools::draw_text(img, fmt::format("tvec:  x{: .5f} y{: .5f}", armor.center.x, armor.center.y), cv::Point2f(5, 30), 1.0, cv::Scalar(0, 255, 255), 2.0);
            tools::draw_text(img, fmt::format("rvec:  x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), cv::Point2f(5, 60), 0.7, cv::Scalar(0, 255, 255), 2.0);
            //
            // 提示：
            // - 使用 tvec.at<double>(0)，可以得到一个double变量，它是tvec中首个元素的值。
            // #########################################################


            // #### Task 05 ############################################
            // 使用 cv::Rodrigues ，把 rvec 旋转向量转换为 rmat 旋转矩阵。
            // 再使用反三角函数，把旋转矩阵 rmat 中的元素转化为欧拉角，并在画面上显示。
            //
            // cv::Mat rmat;
            // cv::Rodrigues(rvec, rmat);
            // double yaw = std::atan2(rmat.at<double>(0,2),rmat.at<double>(2,2));
            // double pitch = std::asin(rmat.at<double>(1,2)) * (-1);
            // double roll = std::atan2(rmat.at<double>(1,0), rmat.at<double>(1,1));

            //tools::draw_text(img, fmt::format("euler angles:  yaw{: .2f} pitch{: .2f} roll{: .2f}", yaw*57.3, pitch*57.3, roll*57.3), cv::Point2f(5, 90), 0.7, cv::Scalar(0, 255, 255), 2.0);
            //
            // 提示：
            // - cv::Mat 的下标从0开始，而不是1。
            // - 从cv::Mat 中取元素的方法和上面的 tvec 类似。如： rmat.at<double>(0, 2)
            // #########################################################

            //****************************************************** */
            kf.predict(freshrate);    // refresh rate , depends on framerate of the stream
            kf.update(armor_x, armor_y, armor_z);  // update with current measurement and predicted values
            cv::Point3f final_est = kf.getFinalEstimation();
            
            //tools::draw_text(img, fmt::format("diff:  x{: .2f} y{: .2f} z{: .2f}", prediction.x-tvec.at<double>(0), prediction.y-tvec.at<double>(1), prediction.z-tvec.at<double>(2)), cv::Point2f(5, 90), 0.7, cv::Scalar(0, 255, 255), 2.0);
            //std::cout << prediction.x << prediction.y << prediction.z << std::endl;

            double x_ratio = (final_est.x) / (armor_x);
            double y_ratio = (final_est.y) / (armor_y);

            double pred_center_x = armor.center.x * x_ratio;
            double pred_center_y = armor.center.y * y_ratio;

            tools::draw_text(img, fmt::format("tvec:  x{: .5f} y{: .5f} z{: .5f}", armor_x, armor_y, armor_z), cv::Point2f(5, 30), 0.7, cv::Scalar(0, 255, 255), 2.0);
            tools::draw_text(img, fmt::format("pred_tvec:  x{: .5f} y{: .5f} z{: .5f}", final_est.x, final_est.y, final_est.z), cv::Point2f(5, 60), 0.7, cv::Scalar(0, 255, 255), 2.0);
            

            tools::draw_text(img, fmt::format("{}", "x"), cv::Point2f(armor.center.x, armor.center.y), 1.0, cv::Scalar(0, 255, 255), 2.0);
            tools::draw_text(img, fmt::format("{}", "x"), cv::Point2f(pred_center_x, pred_center_y), 1.0, cv::Scalar(255, 0, 255), 2.0);

            
            ////////////////////////////////////////////// ////////////.0f}
            float roundedx = std::round(final_est.x * 10000.0) / 10000;
            float roundedy = std::round(final_est.y * 10000.0) / 10000;
            float roundedz = std::round(final_est.z * 10000.0) / 10000;

            // 与电控通讯

            plate.tvec_y = roundedy;
            plate.tvec_z = roundedz;
            plate.is_detected = 1.0f;
            plate.armor_length = roi_length;
            plate.armor_width = roi_width;
            plate.orientation = angle;

            char sendBuffer[sizeof(armorPlate)];
            //put the updated pos into serial
            ///////////////////////////////////////////////////////////
            memcpy(sendBuffer, &plate, sizeof(armorPlate));

            // 发送数据
            serial.sendData(sendBuffer, sizeof(armorPlate));
        } else {
            plate.is_detected = 0.0f;
            plate.orientation = angle;
            char sendBuffer[sizeof(armorPlate)];
            //put the updated pos into serial
            ///////////////////////////////////////////////////////////
            memcpy(sendBuffer, &plate, sizeof(armorPlate));
            //std::cout << sizeof(armorPlate) << std::endl;    

            //发送数据
            serial.sendData(sendBuffer, sizeof(armorPlate));
        }

        cv::imshow("press q to quit", img);

        if (cv::waitKey(20) == 'q')
            break;
    }

    cv::destroyAllWindows();
    return 0;
}