#pragma once

#include "components.hpp"

#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>

namespace reuv {

class VisualTracker {
public:
  /**
   * @brief Construct a new VisualTracker object
   * @param video_capture The index of the camera to open or a video file path
   */
  template <typename T> VisualTracker(T video_capture = 0) {
    if (!open_camera(video_capture))
      throw std::runtime_error("Failed to open camera\n");
  }

  /**
   * @brief Destroy the VisualTracker object
   */
  ~VisualTracker() { cap_.release(); }

  float frame_center_x_ = 0;
  float offset_ = 0;

  /**
   * @brief Start tracking
   */
  void track() {
    cv::Mat frame;
    while (cap_.read(frame)) {
      try {
        frame_center_x_ = static_cast<float>(frame.cols) / 2;
        // Preprocess the image
        cv::Mat preprocessed_image = preprocess_image(frame);

        // Detect edges
        cv::Mat edges = detect_edges(preprocessed_image);

        // Define ROI points
        std::vector<cv::Point> roi_points = {{0, frame.rows},
                                             {0, frame.rows * 6 / 10},
                                             {frame.cols, frame.rows * 6 / 10},
                                             {frame.cols, frame.rows}};

        // Extract ROI image
        cv::Mat roi_image =
            reuv::basic_components::extract_ROI(edges, roi_points);

        // Clear noise
        cv::Mat cleared_edges = clear_noise(roi_image);

        // Find lanelines
        std::vector<cv::Point2f> lanelines = find_lanelines(cleared_edges);

        // Draw lanelines
        draw_lines(lanelines, frame);

        // Calculate average intersection point
        cv::Point2f average_point =
            compute_average_intersection_point(lanelines);

        offset_ = frame_center_x_ - average_point.x;

        // Display the frame
        cv::circle(frame, average_point, 5, cv::Scalar(0, 255, 255), -1);
        cv::line(frame, cv::Point(average_point.x, 0),
                 cv::Point(average_point.x, frame.rows), cv::Scalar(0, 255, 0),
                 2);
        cv::imshow("Lane Detection", frame);
        if (cv::waitKey(50) == 'q') {
          break;
        }
      } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
      }
    }
  }

private:
  cv::VideoCapture cap_;

  /**
   * @brief Preprocess the image
   * @param image The input image
   * @return The preprocessed image
   */
  cv::Mat preprocess_image(const cv::Mat &image) {
    // 去除噪声
    cv::Mat blurred_image;
    cv::GaussianBlur(image, blurred_image, cv::Size(5, 5), 0);

    // 调整对比度和亮度
    cv::Mat adjusted_image;
    cv::convertScaleAbs(blurred_image, adjusted_image, 1.5, 30);

    return adjusted_image;
  }

  /**
   * @brief Detect edges
   * @param image The input image
   * @return The detected edges
   */
  cv::Mat detect_edges(const cv::Mat &image) {
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat edges;
    cv::Canny(gray_image, edges, 50, 150);

    return edges;
  }

  /**
   * @brief Clear noise
   * @param edges The input edges
   * @return The cleared edges
   */
  cv::Mat clear_noise(const cv::Mat &edges) {
    cv::Mat cleared_edges;
    cv::dilate(edges, cleared_edges, cv::Mat(), cv::Point(-1, -1), 2);
    cv::erode(cleared_edges, cleared_edges, cv::Mat(), cv::Point(-1, -1), 2);

    return cleared_edges;
  }

  /**
   * @brief Find lanelines
   * @param edges The input edges
   * @return The detected lanelines
   */
  std::vector<cv::Point2f> find_lanelines(const cv::Mat &edges) {
    std::vector<cv::Point2f> lines;

    cv::Mat lines_p;
    cv::HoughLinesP(edges, lines_p, 1, CV_PI / 180, 100, 10, 10);

    for (int i = 0; i < lines_p.rows; i++) {
      cv::Vec4i line = lines_p.at<cv::Vec4i>(i);
      float x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];

      // calculate slope and intercept
      float slope = (y2 - y1) / (x2 - x1);
      float intercept = y1 - slope * x1;

      // store slope and intercept in cv::Point2f
      cv::Point2f line_params(slope, intercept);

      if (std::abs(slope) > 0.4 && std::abs(slope) < 0.55)
        // std::cout<<"slope: "<<slope<<"  intercept: "<<intercept<<std::endl;
        lines.push_back(line_params);
    }

    return lines;
  }

  /**
   * @brief Draw lanelines
   * @param lines The detected lanelines
   * @param image The input image
   */
  void draw_lines(const std::vector<cv::Point2f> &lines, cv::Mat &image) {
    for (const auto &line_params : lines) {
      float slope = line_params.x;
      float intercept = line_params.y;

      int x1 = 0;
      int y1 = static_cast<int>(slope * x1 + intercept);
      int x2 = image.cols - 1;
      int y2 = static_cast<int>(slope * x2 + intercept);

      cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2),
               cv::Scalar(0, 0, 255), 2);
    }
  }

  /**
   * @brief Compute the average intersection point
   * @param lines The detected lanelines
   * @return The average intersection point
   */
  cv::Point2f
  compute_average_intersection_point(const std::vector<cv::Point2f> &lines) {
    cv::Point2f intersection_point(0, 0);
    std::vector<cv::Point2f> left_lines, right_lines;
    int good_points = 0;
    for (const auto &line : lines) {
      if (std::abs(line.x) > 0.4 && std::abs(line.x) < 0.6) {
        float slope = line.x;
        (slope < 0 ? left_lines : right_lines).push_back(line);
      }
    }

    for (const auto &left_line : left_lines) {
      for (const auto &right_line : right_lines) {
        float left_slope = left_line.x;
        float left_intercept = left_line.y;
        float right_slope = right_line.x;
        float right_intercept = right_line.y;

        float denominator = left_slope - right_slope;
        if (denominator == 0) {
          continue;
        }

        float x = (right_intercept - left_intercept) / denominator;
        float y = left_slope * x + left_intercept;
        std::cout << "intersection_point:" << x << "," << y << ")" << std::endl;
        if (x > 600 && x < 700) {
          intersection_point += cv::Point2f(x, y);
          good_points++;
        }
      }
    }

    if (left_lines.size() * right_lines.size() > 0) {
      // intersection_point.x /= left_lines.size() * right_lines.size();
      // intersection_point.y /= left_lines.size() * right_lines.size();
      intersection_point.x /= good_points;
      intersection_point.y /= good_points;
      // std::cout<<"ave_intersection_point:"<<intersection_point.x<<","<<intersection_point.y<<")"<<std::endl;
    }

    return intersection_point;
  }

  /**
   * @brief Open a camera or a video file
   * @tparam T The type of the video capture
   * @param video_capture The index of the camera to open or a video file path
   * @return Whether the camera or video file is opened successfully
   */
  template <typename T> bool open_camera(T video_capture) {
    cap_ = cv::VideoCapture(video_capture);
    if (!cap_.isOpened())
      return 0;
    return 1;
  }
};
} // namespace reuv