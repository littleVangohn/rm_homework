#include "image_publisher/image_publisher_node.hpp"

namespace image_publisher
{

// 静态常量定义
const std::vector<std::string> ImagePublisherNode::SUPPORTED_EXTENSIONS = {
    ".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"
};

ImagePublisherNode::ImagePublisherNode() 
    : Node("image_publisher_node"), 
      current_image_index_(0)
{
    // 初始化参数
    initializeParameters();
    
    // 创建发布者
    createPublishers();
    
    // 加载图片文件列表
    if (!loadImageFiles()) {
        RCLCPP_ERROR(this->get_logger(), "无法加载图片文件，节点将退出");
        rclcpp::shutdown();
        return;
    }
    
    // 预加载图像到内存（可选）
    if (preload_images_) {
        RCLCPP_INFO(this->get_logger(), "预加载图像到内存中...");
        preloadImages();
        RCLCPP_INFO(this->get_logger(), "图像预加载完成");
    }
    
    // 创建定时器
    double publish_rate = this->get_parameter("publish_rate").as_double();
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(timer_period, std::bind(&ImagePublisherNode::publishImage, this));
    
    // 打印节点信息
    printNodeInfo();
}

void ImagePublisherNode::initializeParameters()
{
    // 声明参数
    this->declare_parameter("image_folder", "");
    this->declare_parameter("publish_rate", 1.0);
    this->declare_parameter("loop", false);  // 默认不循环播放
    this->declare_parameter("preload_images", true);  // 默认预加载，确保每张图片只读取一次
    this->declare_parameter("use_compression", false);
    this->declare_parameter("queue_size", 50);
    this->declare_parameter("resize_width", 0);
    this->declare_parameter("resize_height", 0);
    
    // 获取参数
    image_folder_ = this->get_parameter("image_folder").as_string();
    loop_images_ = this->get_parameter("loop").as_bool();
    preload_images_ = this->get_parameter("preload_images").as_bool();
    use_compression_ = this->get_parameter("use_compression").as_bool();
    resize_width_ = this->get_parameter("resize_width").as_int();
    resize_height_ = this->get_parameter("resize_height").as_int();
    
    // 如果没有指定图片文件夹，使用默认路径
    if (image_folder_.empty()) {
        // 尝试使用包内的images文件夹
        image_folder_ = std::filesystem::current_path() / "src/image_publisher/images";
        RCLCPP_INFO(this->get_logger(), "使用默认图片文件夹: %s", image_folder_.c_str());
    }
}

void ImagePublisherNode::createPublishers()
{
    int queue_size = this->get_parameter("queue_size").as_int();
    
    // 创建发布者 - 使用更大的队列深度
    if (use_compression_) {
        compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/sensor_img/compressed", queue_size);
        RCLCPP_INFO(this->get_logger(), "使用压缩图像发布");
    } else {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/sensor_img", queue_size);
        RCLCPP_INFO(this->get_logger(), "使用原始图像发布");
    }
}

void ImagePublisherNode::printNodeInfo()
{
    double publish_rate = this->get_parameter("publish_rate").as_double();
    int queue_size = this->get_parameter("queue_size").as_int();
    
    RCLCPP_INFO(this->get_logger(), "图像发布节点已启动");
    RCLCPP_INFO(this->get_logger(), "图片文件夹: %s", image_folder_.c_str());
    RCLCPP_INFO(this->get_logger(), "找到 %zu 个图片文件", image_files_.size());
    RCLCPP_INFO(this->get_logger(), "发布频率: %.1f Hz", publish_rate);
    RCLCPP_INFO(this->get_logger(), "循环播放: %s", loop_images_ ? "是" : "否");
    RCLCPP_INFO(this->get_logger(), "预加载图像: %s", preload_images_ ? "是（每张图片只读取一次）" : "否");
    RCLCPP_INFO(this->get_logger(), "队列大小: %d", queue_size);
    
    if (!loop_images_) {
        RCLCPP_INFO(this->get_logger(), "注意：循环播放已禁用，所有图片发布完毕后节点将自动退出");
    }
    if (resize_width_ > 0 && resize_height_ > 0) {
        RCLCPP_INFO(this->get_logger(), "图像缩放至: %dx%d", resize_width_, resize_height_);
    }
}

bool ImagePublisherNode::loadImageFiles()
{
    if (!std::filesystem::exists(image_folder_)) {
        RCLCPP_ERROR(this->get_logger(), "图片文件夹不存在: %s", image_folder_.c_str());
        return false;
    }
    
    try {
        for (const auto& entry : std::filesystem::directory_iterator(image_folder_)) {
            if (entry.is_regular_file()) {
                std::string extension = entry.path().extension().string();
                std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
                
                if (std::find(SUPPORTED_EXTENSIONS.begin(), SUPPORTED_EXTENSIONS.end(), extension) 
                    != SUPPORTED_EXTENSIONS.end()) {
                    image_files_.push_back(entry.path().string());
                }
            }
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        RCLCPP_ERROR(this->get_logger(), "读取文件夹时出错: %s", ex.what());
        return false;
    }
    
    if (image_files_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "在文件夹 %s 中没有找到支持的图片文件", image_folder_.c_str());
        return false;
    }
    
    // 对文件名进行排序
    std::sort(image_files_.begin(), image_files_.end());
    return true;
}

void ImagePublisherNode::preloadImages()
{
    preloaded_images_.reserve(image_files_.size());
    
    for (const auto& image_path : image_files_) {
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (!image.empty()) {
            // 如果设置了缩放参数，则缩放图像
            if (resize_width_ > 0 && resize_height_ > 0) {
                cv::Mat resized_image;
                cv::resize(image, resized_image, cv::Size(resize_width_, resize_height_));
                preloaded_images_.push_back(resized_image);
            } else {
                preloaded_images_.push_back(image);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "无法预加载图片: %s", image_path.c_str());
            // 添加空图像占位符
            preloaded_images_.push_back(cv::Mat());
        }
    }
}

void ImagePublisherNode::publishImage()
{
    if (image_files_.empty()) {
        return;
    }
    
    // 检查是否需要循环
    if (current_image_index_ >= image_files_.size()) {
        if (loop_images_) {
            current_image_index_ = 0;
            RCLCPP_INFO(this->get_logger(), "重新开始循环播放图片");
        } else {
            RCLCPP_INFO(this->get_logger(), "所有图片已发布完毕（共%zu张），节点即将退出", image_files_.size());
            timer_->cancel();
            // 延迟一小段时间后退出，确保最后的消息能被发送
            auto exit_timer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() { rclcpp::shutdown(); }
            );
            return;
        }
    }
    
    cv::Mat image;
    
    // 获取图像数据
    if (preload_images_ && current_image_index_ < preloaded_images_.size()) {
        image = preloaded_images_[current_image_index_];
    } else {
        // 实时读取图片
        std::string current_image_path = image_files_[current_image_index_];
        image = cv::imread(current_image_path, cv::IMREAD_COLOR);
        
        // 如果设置了缩放参数，则缩放图像
        if (!image.empty() && resize_width_ > 0 && resize_height_ > 0) {
            cv::Mat resized_image;
            cv::resize(image, resized_image, cv::Size(resize_width_, resize_height_));
            image = resized_image;
        }
    }
    
    if (image.empty()) {
        RCLCPP_WARN(this->get_logger(), "无法读取图片: %s", 
                   image_files_[current_image_index_].c_str());
        current_image_index_++;
        return;
    }
    
    // 转换为ROS图像消息
    try {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";
        
        if (use_compression_) {
            // 发布压缩图像
            publishCompressedImage(image, header);
        } else {
            // 发布原始图像
            cv_bridge::CvImage cv_image(header, "bgr8", image);
            sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();
            publisher_->publish(*msg);
        }
        
        RCLCPP_INFO(this->get_logger(), "发布图片: %s [%zu/%zu]", 
                    std::filesystem::path(image_files_[current_image_index_]).filename().c_str(),
                    current_image_index_ + 1, image_files_.size());
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
    }
    
    current_image_index_++;
}

void ImagePublisherNode::publishCompressedImage(const cv::Mat& image, const std_msgs::msg::Header& header)
{
    auto compressed_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    compressed_msg->header = header;
    compressed_msg->format = "jpeg";
    
    // 压缩图像
    std::vector<uchar> buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90}; // 90%质量
    cv::imencode(".jpg", image, buffer, compression_params);
    
    compressed_msg->data = buffer;
    compressed_publisher_->publish(std::move(compressed_msg));
}

} // namespace image_publisher

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<image_publisher::ImagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 