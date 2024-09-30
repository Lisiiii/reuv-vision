#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class VisualTracker {
public:
    template <typename T>
    VisualTracker(T video_capture = 0)
    {
        if (!open_camera(video_capture))
            std::cout << "Failed to open camera\n";
    }
    ~VisualTracker()
    {
        cap_.release();
    }

    void tracking()
    {
        // 读取视频帧
        cv::Mat frame;
        while (cap_.read(frame)) {
            // 预处理图像
            cv::Mat preprocessed_image = preprocess_image(frame);
            // 检测边缘
            cv::Mat edges = detect_edges(preprocessed_image);
            // 清除噪声
            cv::Mat cleared_edges = clear_noise(edges);
            // 检测直线
            std::vector<cv::Vec4i> lines = detect_lines(cleared_edges);
            // 拟合车道线
            std::vector<cv::Vec4i> lane_lines = fit_lanelines(lines);
            // 计算车辆离车道线的位置
            cv::Point distance_to_lane = calculate_distance_to_lane(frame, lane_lines);
            // 显示结果
            cv::circle(frame, distance_to_lane, 5, cv::Scalar(0, 0, 255), -1);
            cv::imshow("Lane Detection", frame);
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }
    };

private:
    cv::VideoCapture cap_;

    // 预处理图像
    cv::Mat preprocess_image(const cv::Mat& image)
    {
        // 去除噪声
        cv::Mat blurred_image;
        cv::GaussianBlur(image, blurred_image, cv::Size(5, 5), 0);

        // 调整对比度和亮度
        cv::Mat adjusted_image;
        cv::convertScaleAbs(blurred_image, adjusted_image, 1.5, 30);

        return adjusted_image;
    }

    // 检测边缘
    cv::Mat detect_edges(const cv::Mat& image)
    {
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        cv::Mat edges;
        cv::Canny(gray_image, edges, 50, 150);

        return edges;
    }

    // 清除噪声
    cv::Mat clear_noise(const cv::Mat& edges)
    {
        cv::Mat cleared_edges;
        cv::dilate(edges, cleared_edges, cv::Mat(), cv::Point(-1, -1), 2);
        cv::erode(cleared_edges, cleared_edges, cv::Mat(), cv::Point(-1, -1), 2);

        return cleared_edges;
    }

    // 检测直线
    std::vector<cv::Vec4i> detect_lines(const cv::Mat& edges)
    {
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

        return lines;
    }

    // 拟合车道线
    std::vector<cv::Vec4i> fit_lanelines(const std::vector<cv::Vec4i>& lines)
    {
        std::vector<cv::Vec4i> lane_lines;

        // 选择可能是车道线的直线
        for (const auto& line : lines) {
            // 计算直线的斜率
            double slope = (line[3] - line[1]) / (double)(line[2] - line[0]);

            // 选择斜率在一定范围内的直线
            if (slope > 0.5 && slope < 2.0) {
                lane_lines.push_back(line);
            }
        }

        return lane_lines;
    }

    // 计算车辆离车道线的位置
    cv::Point calculate_distance_to_lane(const cv::Mat& image, const std::vector<cv::Vec4i>& lane_lines)
    {
        // 计算车辆中心的位置
        cv::Point vehicle_center(image.cols / 2, image.rows / 2);

        // 计算车辆离车道线的位置
        cv::Point distance_to_lane(0, 0);

        // 计算车辆离每条车道线的距离
        for (const auto& line : lane_lines) {
            // 计算直线的方程
            double slope = (line[3] - line[1]) / (double)(line[2] - line[0]);
            double intercept = line[1] - slope * line[0];

            // 计算车辆中心到直线的距离
            double distance = std::abs(slope * vehicle_center.x - vehicle_center.y + intercept) / std::sqrt(slope * slope + 1.0);

            // 更新车辆离车道线的位置
            distance_to_lane.x += distance;
            distance_to_lane.y += slope;
        }

        // 计算平均距离
        distance_to_lane.x /= lane_lines.size();
        distance_to_lane.y /= lane_lines.size();

        return distance_to_lane;
    }

    template <typename T>
    bool open_camera(T video_capture)
    {
        cap_ = cv::VideoCapture(video_capture);
        if (!cap_.isOpened())
            return 0;
        return 1;
    }
};
