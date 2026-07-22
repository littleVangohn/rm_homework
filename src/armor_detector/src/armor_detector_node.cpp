#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <aim_interfaces/msg/aim_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace armor_detector
{

struct Detection
{
  cv::Rect box;
  int class_id{0};
  float confidence{0.0F};
};

struct LightBar
{
  cv::Point2f center;
  cv::Point2f top;
  cv::Point2f bottom;
  float length{0.0F};
  float width{0.0F};
  float angle{0.0F};
};

struct ArmorCorners
{
  std::array<cv::Point2f, 4> points{};  // top-left, top-right, bottom-right, bottom-left
  float score{std::numeric_limits<float>::max()};
  bool from_light_bars{false};
};

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode()
  : Node("armor_detector"), ort_env_(ORT_LOGGING_LEVEL_WARNING, "armor_detector")
  {
    declareParameters();
    loadParameters();

    if (model_path_.empty()) {
      model_path_ = ament_index_cpp::get_package_share_directory("armor_detector") +
        "/models/armor_yolo11s.onnx";
    }

    try {
      ort_options_.SetIntraOpNumThreads(2);
      ort_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      ort_session_ = std::make_unique<Ort::Session>(ort_env_, model_path_.c_str(), ort_options_);
      Ort::AllocatorWithDefaultOptions allocator;
      input_name_ = ort_session_->GetInputNameAllocated(0, allocator).get();
      output_name_ = ort_session_->GetOutputNameAllocated(0, allocator).get();
    } catch (const Ort::Exception & error) {
      RCLCPP_FATAL(get_logger(), "Failed to load ONNX model '%s': %s", model_path_.c_str(), error.what());
      throw;
    }

    aim_publisher_ = create_publisher<aim_interfaces::msg::AimInfo>("/aim_target", 10);
    debug_publisher_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, 10);
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/sensor_img", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Ready: /sensor_img -> /aim_target and %s (coordinates: robot frame, mm)",
      debug_topic_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("model_path", "");
    declare_parameter<int>("input_size", 960);
    declare_parameter<double>("confidence_threshold", 0.25);
    declare_parameter<double>("nms_threshold", 0.45);
    declare_parameter<double>("armor_width", 0.16);
    declare_parameter<double>("armor_height", 0.08);
    declare_parameter<std::vector<double>>(
      "camera_matrix", {1462.3697, 0.0, 398.59394, 0.0, 1469.68385, 110.68997, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>(
      "distortion", {0.003518, -0.311778, -0.016581, 0.023682, 0.0});
    declare_parameter<std::vector<double>>("camera_translation", {0.08, 0.0, 0.05});
    declare_parameter<std::vector<double>>("camera_rpy_deg", {0.0, 60.0, 20.0});
    declare_parameter<bool>("use_ros_optical_convention", true);
    declare_parameter<double>("coordinate_scale", 1000.0);
    declare_parameter<std::string>("debug_topic", "/aim_debug");
  }

  static void requireSize(const std::string & name, const std::vector<double> & values, size_t size)
  {
    if (values.size() != size) {
      throw std::runtime_error("Parameter '" + name + "' must contain " + std::to_string(size) + " values");
    }
  }

  void loadParameters()
  {
    model_path_ = get_parameter("model_path").as_string();
    input_size_ = get_parameter("input_size").as_int();
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    nms_threshold_ = static_cast<float>(get_parameter("nms_threshold").as_double());
    armor_width_ = get_parameter("armor_width").as_double();
    armor_height_ = get_parameter("armor_height").as_double();
    coordinate_scale_ = get_parameter("coordinate_scale").as_double();
    use_ros_optical_convention_ = get_parameter("use_ros_optical_convention").as_bool();
    debug_topic_ = get_parameter("debug_topic").as_string();

    const auto intrinsics = get_parameter("camera_matrix").as_double_array();
    const auto distortion = get_parameter("distortion").as_double_array();
    camera_translation_ = get_parameter("camera_translation").as_double_array();
    const auto rpy_deg = get_parameter("camera_rpy_deg").as_double_array();
    requireSize("camera_matrix", intrinsics, 9);
    requireSize("distortion", distortion, 5);
    requireSize("camera_translation", camera_translation_, 3);
    requireSize("camera_rpy_deg", rpy_deg, 3);

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    std::copy(intrinsics.begin(), intrinsics.end(), camera_matrix_.ptr<double>());
    distortion_ = cv::Mat(1, 5, CV_64F);
    std::copy(distortion.begin(), distortion.end(), distortion_.ptr<double>());
    robot_from_camera_ = eulerMatrix(rpy_deg[0], rpy_deg[1], rpy_deg[2]);
  }

  static cv::Matx33d eulerMatrix(double roll_deg, double pitch_deg, double yaw_deg)
  {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double roll = roll_deg * kDegToRad;
    const double pitch = pitch_deg * kDegToRad;
    const double yaw = yaw_deg * kDegToRad;
    const cv::Matx33d rx(
      1, 0, 0, 0, std::cos(roll), -std::sin(roll), 0, std::sin(roll), std::cos(roll));
    const cv::Matx33d ry(
      std::cos(pitch), 0, std::sin(pitch), 0, 1, 0, -std::sin(pitch), 0, std::cos(pitch));
    const cv::Matx33d rz(
      std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(yaw), 0, 0, 0, 1);
    return rz * ry * rx;
  }

  cv::Mat letterbox(const cv::Mat & image, float & scale, int & pad_x, int & pad_y) const
  {
    scale = std::min(
      static_cast<float>(input_size_) / static_cast<float>(image.cols),
      static_cast<float>(input_size_) / static_cast<float>(image.rows));
    const int resized_width = static_cast<int>(std::round(image.cols * scale));
    const int resized_height = static_cast<int>(std::round(image.rows * scale));
    pad_x = (input_size_ - resized_width) / 2;
    pad_y = (input_size_ - resized_height) / 2;
    cv::Mat canvas(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_width, resized_height));
    resized.copyTo(canvas(cv::Rect(pad_x, pad_y, resized_width, resized_height)));
    return canvas;
  }

  std::vector<Detection> infer(const cv::Mat & image)
  {
    float scale = 1.0F;
    int pad_x = 0;
    int pad_y = 0;
    const cv::Mat padded = letterbox(image, scale, pad_x, pad_y);
    const cv::Mat blob = cv::dnn::blobFromImage(
      padded, 1.0 / 255.0, cv::Size(input_size_, input_size_), cv::Scalar(), true, false);
    const std::array<int64_t, 4> input_shape = {1, 3, input_size_, input_size_};
    const Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
      OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, reinterpret_cast<float *>(blob.data), blob.total(),
      input_shape.data(), input_shape.size());
    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};
    std::vector<Ort::Value> outputs = ort_session_->Run(
      Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
    const std::vector<int64_t> output_shape =
      outputs.front().GetTensorTypeAndShapeInfo().GetShape();
    const float * output = outputs.front().GetTensorData<float>();

    constexpr int dimensions = 9;  // cx, cy, w, h and five class probabilities
    if (output_shape.size() != 3 ||
      (output_shape[1] != dimensions && output_shape[2] != dimensions))
    {
      throw std::runtime_error("Unexpected YOLO output shape; expected [1,9,N] or [1,N,9]");
    }
    const bool channels_first = output_shape[1] == dimensions;
    const int row_count = static_cast<int>(channels_first ? output_shape[2] : output_shape[1]);
    const auto value_at = [output, channels_first, row_count](int row, int column) {
        return channels_first ? output[column * row_count + row] : output[row * dimensions + column];
      };

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    for (int row = 0; row < row_count; ++row) {
      int class_id = 0;
      float confidence = value_at(row, 4);
      for (int class_index = 1; class_index < 5; ++class_index) {
        if (value_at(row, 4 + class_index) > confidence) {
          confidence = value_at(row, 4 + class_index);
          class_id = class_index;
        }
      }
      if (confidence < confidence_threshold_) {
        continue;
      }
      const float left = (value_at(row, 0) - 0.5F * value_at(row, 2) - pad_x) / scale;
      const float top = (value_at(row, 1) - 0.5F * value_at(row, 3) - pad_y) / scale;
      const float width = value_at(row, 2) / scale;
      const float height = value_at(row, 3) / scale;
      cv::Rect box(
        static_cast<int>(std::round(left)), static_cast<int>(std::round(top)),
        static_cast<int>(std::round(width)), static_cast<int>(std::round(height)));
      box &= cv::Rect(0, 0, image.cols, image.rows);
      if (box.area() > 0) {
        boxes.push_back(box);
        confidences.push_back(confidence);
        class_ids.push_back(class_id);
      }
    }

    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, kept);
    std::vector<Detection> detections;
    detections.reserve(kept.size());
    for (const int index : kept) {
      detections.push_back({boxes[index], class_ids[index], confidences[index]});
    }
    std::sort(detections.begin(), detections.end(), [](const Detection & left, const Detection & right) {
      return left.confidence > right.confidence;
    });
    return detections;
  }

  static std::vector<LightBar> findLightBars(const cv::Mat & image)
  {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat mask;
    cv::threshold(gray, mask, 150, 255, cv::THRESH_BINARY);
    cv::morphologyEx(
      mask, mask, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<LightBar> bars;
    for (const auto & contour : contours) {
      if (cv::contourArea(contour) < 8.0) {
        continue;
      }
      const cv::RotatedRect rectangle = cv::minAreaRect(contour);
      float major_angle = rectangle.angle;
      float length = rectangle.size.width;
      float width = rectangle.size.height;
      if (length < width) {
        std::swap(length, width);
        major_angle += 90.0F;
      }
      if (width < 1.0F || length < 8.0F || length / width < 2.0F || length / width > 25.0F) {
        continue;
      }
      const float radians = major_angle * static_cast<float>(CV_PI / 180.0);
      cv::Point2f axis(std::cos(radians), std::sin(radians));
      if (std::abs(axis.y) < 0.5F) {
        continue;
      }
      cv::Point2f first = rectangle.center - axis * (0.5F * length);
      cv::Point2f second = rectangle.center + axis * (0.5F * length);
      if (first.y > second.y) {
        std::swap(first, second);
      }
      bars.push_back({rectangle.center, first, second, length, width, major_angle});
    }
    return bars;
  }

  static bool pointNearPair(const cv::Point2f & point, const LightBar & left, const LightBar & right)
  {
    const float average_length = 0.5F * (left.length + right.length);
    return point.x > left.center.x - 0.2F * average_length &&
           point.x < right.center.x + 0.2F * average_length &&
           point.y > std::min(left.top.y, right.top.y) - 0.5F * average_length &&
           point.y < std::max(left.bottom.y, right.bottom.y) + 0.5F * average_length;
  }

  static bool findArmorFromLights(
    const std::vector<LightBar> & bars, const cv::Point2f * preferred_center, ArmorCorners & result)
  {
    bool found = false;
    for (size_t i = 0; i < bars.size(); ++i) {
      for (size_t j = i + 1; j < bars.size(); ++j) {
        const LightBar & left = bars[i].center.x < bars[j].center.x ? bars[i] : bars[j];
        const LightBar & right = bars[i].center.x < bars[j].center.x ? bars[j] : bars[i];
        const float average_length = 0.5F * (left.length + right.length);
        const float separation = right.center.x - left.center.x;
        const float ratio = separation / average_length;
        const float y_error = std::abs(left.center.y - right.center.y) / average_length;
        const float length_ratio = std::max(left.length, right.length) / std::min(left.length, right.length);
        if (ratio < 0.7F || ratio > 4.5F || y_error > 0.65F || length_ratio > 1.8F) {
          continue;
        }
        if (preferred_center != nullptr && !pointNearPair(*preferred_center, left, right)) {
          continue;
        }
        const float pair_center_x = 0.5F * (left.center.x + right.center.x);
        const float pair_center_y = 0.5F * (left.center.y + right.center.y);
        float score = std::abs(ratio - 2.0F) + y_error + (length_ratio - 1.0F);
        if (preferred_center != nullptr) {
          score += 0.5F * std::hypot(
            preferred_center->x - pair_center_x, preferred_center->y - pair_center_y) / average_length;
        }
        if (score < result.score) {
          result.points = {left.top, right.top, right.bottom, left.bottom};
          result.score = score;
          result.from_light_bars = true;
          found = true;
        }
      }
    }
    return found;
  }

  static ArmorCorners cornersForDetection(const Detection & detection)
  {
    ArmorCorners corners;
    const float left = static_cast<float>(detection.box.x);
    const float top = static_cast<float>(detection.box.y);
    const float right = static_cast<float>(detection.box.x + detection.box.width);
    const float bottom = static_cast<float>(detection.box.y + detection.box.height);
    corners.points = {
      cv::Point2f(left, top), cv::Point2f(right, top),
      cv::Point2f(right, bottom), cv::Point2f(left, bottom)};
    corners.from_light_bars = false;
    return corners;
  }

  bool estimateRobotPosition(const ArmorCorners & corners, cv::Vec3d & robot_point) const
  {
    const double half_width = armor_width_ / 2.0;
    const double half_height = armor_height_ / 2.0;
    const std::vector<cv::Point3d> object_points = {
      {-half_width, -half_height, 0.0}, {half_width, -half_height, 0.0},
      {half_width, half_height, 0.0}, {-half_width, half_height, 0.0}};
    const std::vector<cv::Point2f> image_points(corners.points.begin(), corners.points.end());
    cv::Vec3d rotation_vector;
    cv::Vec3d translation_vector;
    if (!cv::solvePnP(
        object_points, image_points, camera_matrix_, distortion_, rotation_vector,
        translation_vector, false, cv::SOLVEPNP_IPPE))
    {
      return false;
    }
    if (translation_vector[2] <= 0.0 || !cv::checkRange(cv::Mat(translation_vector))) {
      return false;
    }

    cv::Vec3d camera_link_point = translation_vector;
    if (use_ros_optical_convention_) {
      // REP-103: optical (right, down, forward) -> camera_link (forward, left, up).
      camera_link_point = cv::Vec3d(
        translation_vector[2], -translation_vector[0], -translation_vector[1]);
    }
    robot_point = robot_from_camera_ * camera_link_point + cv::Vec3d(
      camera_translation_[0], camera_translation_[1], camera_translation_[2]);
    return cv::checkRange(cv::Mat(robot_point));
  }

  int16_t scaledCoordinate(double meters) const
  {
    const double value = std::round(meters * coordinate_scale_);
    return static_cast<int16_t>(std::clamp(
      value, static_cast<double>(std::numeric_limits<int16_t>::min()),
      static_cast<double>(std::numeric_limits<int16_t>::max())));
  }

  void publishTarget(
    cv::Mat & debug, const ArmorCorners & corners, int type, float confidence,
    const cv::Vec3d & robot_point) const
  {
    aim_interfaces::msg::AimInfo message;
    message.coordinate = {
      scaledCoordinate(robot_point[0]), scaledCoordinate(robot_point[1]),
      scaledCoordinate(robot_point[2])};
    message.type = static_cast<int16_t>(type);
    aim_publisher_->publish(message);

    const cv::Point text_origin(
      std::max(0, static_cast<int>(corners.points[0].x)),
      std::max(24, static_cast<int>(corners.points[0].y) - 8));
    const std::string text = "type=" + std::to_string(type) + " conf=" +
      cv::format("%.2f", confidence) + " xyz_mm=[" +
      std::to_string(message.coordinate[0]) + "," + std::to_string(message.coordinate[1]) + "," +
      std::to_string(message.coordinate[2]) + "]";
    cv::putText(debug, text, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 4);
    cv::putText(debug, text, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    cv::Mat image;
    try {
      image = cv_bridge::toCvShare(message, "bgr8")->image;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", error.what());
      return;
    }
    cv::Mat debug = image.clone();

    try {
      const std::vector<Detection> detections = infer(image);
      bool target_published = false;

      if (!detections.empty()) {
        const Detection & target = detections.front();
        const ArmorCorners corners = cornersForDetection(target);
        cv::Vec3d robot_point;
        if (estimateRobotPosition(corners, robot_point)) {
          publishTarget(debug, corners, target.class_id + 1, target.confidence, robot_point);
          target_published = true;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000, "Target type=%d at [%.3f, %.3f, %.3f] m",
            target.class_id + 1, robot_point[0], robot_point[1], robot_point[2]);
        }
        for (const auto & detection : detections) {
          cv::rectangle(debug, detection.box, cv::Scalar(255, 180, 0), 2);
          cv::putText(
            debug, std::to_string(detection.class_id + 1), detection.box.tl() + cv::Point(0, -3),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 180, 0), 2);
        }
      }

      if (!target_published) {
        cv::putText(
          debug, "NO TARGET", cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8,
          cv::Scalar(0, 0, 255), 2);
      }
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR(get_logger(), "OpenCV inference failed: %s", error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Detection failed: %s", error.what());
    }

    auto debug_message = cv_bridge::CvImage(message->header, "bgr8", debug).toImageMsg();
    debug_publisher_->publish(*debug_message);
  }

  Ort::Env ort_env_;
  Ort::SessionOptions ort_options_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::string input_name_;
  std::string output_name_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  cv::Matx33d robot_from_camera_{};
  std::vector<double> camera_translation_;
  std::string model_path_;
  std::string debug_topic_;
  int input_size_{960};
  float confidence_threshold_{0.25F};
  float nms_threshold_{0.45F};
  double armor_width_{0.16};
  double armor_height_{0.08};
  double coordinate_scale_{1000.0};
  bool use_ros_optical_convention_{true};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<aim_interfaces::msg::AimInfo>::SharedPtr aim_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
};

}  // namespace armor_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<armor_detector::ArmorDetectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("armor_detector"), "Node startup failed: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
