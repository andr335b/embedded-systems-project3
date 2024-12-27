#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <termios.h>

#define BRAM0_BASEADDR 0xA0000000  // BRAM 0 base address
#define BRAM1_BASEADDR 0xA0002000  // BRAM 1 base address
#define PAGE_SIZE 4096             
#define NUM_INPUTS 100             // 10x10 grayscale image pixels
#define UART_PORT "/dev/ttyPS0"    


//code for setting up the ROS node
//It subscribes to the raw_images from the USB_CAM_NODE
//Handles the image and sends it down into BRAM0
//Then it reads from BRAM1 for the prediction
class ImageProcessorNode : public rclcpp::Node {
public:
    ImageProcessorNode();
    ~ImageProcessorNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void process_image(const cv::Mat& image);
    void save_image(const cv::Mat& image, const std::string& path);
    void send_to_uart(const std::string& data);
    void update_popular_number(uint8_t value);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    int fd_;
    uint32_t* bram0_;
    uint32_t* bram1_;
    int uart_fd_;
    std::vector<uint8_t> recent_values_;
    std::mutex processing_mutex_;
};

ImageProcessorNode::ImageProcessorNode()
    : Node("image_processor_node"),
      fd_(-1),
      bram0_(nullptr),
      bram1_(nullptr),
      uart_fd_(-1)
{
    RCLCPP_INFO(this->get_logger(), "Welcome to summoners rift");

    fd_ = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Remember to run with sudo ");
        return;
    }

/*
Running with sudo: 
export AMENT_PREFIX_PATH=/home/mp4d/ros2_ws/install/image_processor:$AMENT_PREFIX_PATH
sudo env AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH ROS_DOMAIN_ID=69 bash -c "source /opt/ros/foxy/setup.bash && source /home/mp4d/ros2_ws/install/setup.bash && ros2 run image_processor image_processor_node"

*/

    bram0_ = (uint32_t*)mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, BRAM0_BASEADDR);
    bram1_ = (uint32_t*)mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, BRAM1_BASEADDR);

    if (bram0_ == MAP_FAILED || bram1_ == MAP_FAILED) {
        RCLCPP_ERROR(this->get_logger(), "Who needs a map!... Us :c");
        close(fd_);
        fd_ = -1;
        return;
    }

    uart_fd_ = open(UART_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UART port: %s", UART_PORT);
        return;
    }

    struct termios tty;
    if (tcgetattr(uart_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "YOu fucked the UART");
        close(uart_fd_);
        uart_fd_ = -1;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; 
    tty.c_iflag &= ~IGNBRK;                    
    tty.c_lflag = 0;                        
    tty.c_oflag = 0;                          
    tty.c_cc[VMIN] = 1;                        
    tty.c_cc[VTIME] = 1;                       

    if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "UART got jayced.");
        close(uart_fd_);
        uart_fd_ = -1;
        return;
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));

    recent_values_.reserve(20);

    RCLCPP_INFO(this->get_logger(), "Its working! ITS WORKING -anakin .");
}

ImageProcessorNode::~ImageProcessorNode()
{
    if (uart_fd_ != -1) {
        close(uart_fd_);
    }

    if (bram0_ != nullptr && bram0_ != MAP_FAILED) {
        munmap(bram0_, PAGE_SIZE);
    }
    if (bram1_ != nullptr && bram1_ != MAP_FAILED) {
        munmap(bram1_, PAGE_SIZE);
    }
    if (fd_ != -1) {
        close(fd_);
    }

    RCLCPP_INFO(this->get_logger(), "Its over");
}

void ImageProcessorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Received an image.");

    if (fd_ == -1 || bram0_ == nullptr || bram1_ == nullptr ||
        bram0_ == MAP_FAILED || bram1_ == MAP_FAILED) {
        RCLCPP_ERROR(this->get_logger(), "Memory not properly initialized. What the fuck are you doing?");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;
    if (img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Imposter image (no data )");
        return;
    }
    
    save_image(img, "/home/mp4d/code_stuff/temp_images/original_image2.jpg");


    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    //cv::bitwise_not(img, img);

    process_image(img);
}

void ImageProcessorNode::process_image(const cv::Mat& image)
{
    std::lock_guard<std::mutex> lock(processing_mutex_);


    save_image(image, "/home/mp4d/code_stuff/temp_images/original_image.jpg");

    cv::Mat cropped_img = image(cv::Rect(0, 0, std::min(500, image.cols), std::min(500, image.rows)));
    save_image(cropped_img, "/home/mp4d/code_stuff/temp_images/cropped_image.jpg");


    cv::Mat resized_img;
    cv::resize(cropped_img, resized_img, cv::Size(10, 10), 0, 0, cv::INTER_AREA);


//Image processing for better results from AI
    cv::Scalar mean_val = cv::mean(resized_img);
    uint8_t threshold = static_cast<uint8_t>(std::max(5.0, mean_val[0]*1.2)); 

    cv::Mat mask = resized_img >= 5;  
    cv::Mat binary_img = resized_img.clone();
    binary_img.setTo(0); 
    resized_img.copyTo(binary_img, mask);  
    binary_img.setTo(0, binary_img <= threshold); 




    save_image(resized_img, "/home/mp4d/code_stuff/temp_images/resized_image.jpg");
    save_image(binary_img, "/home/mp4d/code_stuff/temp_images/resized_image2.jpg");

//We can switch between the two options, usually the binary_img performs better


    float normalized_img[NUM_INPUTS];
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            normalized_img[i * 10 + j] =binary_img.at<uint8_t>(i, j) / 255.0f;
            bram0_[i * 10 + j] = *reinterpret_cast<uint32_t*>(&normalized_img[i * 10 + j]);
        }
    }

    usleep(200000); // 200 ms

    uint32_t bram1_value = bram1_[0];
    uint8_t processed_value = bram1_value & 0xFF;

    //RCLCPP_INFO(this->get_logger(), "Value from BRAM1: %d", processed_value);
    if(processed_value == 0){
        processed_value = 21;
    }
    update_popular_number(processed_value);
}

void ImageProcessorNode::save_image(const cv::Mat& image, const std::string& path)
{
    try {
        if (!cv::imwrite(path, image)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save image to %s", path.c_str());
        } else {
           // RCLCPP_INFO(this->get_logger(), "Image saved to %s", path.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while saving image to %s: %s", path.c_str(), e.what());
    }
}

void ImageProcessorNode::update_popular_number(uint8_t value)
{
    if (recent_values_.size() >= 10) {
        recent_values_.erase(recent_values_.begin());
    }
    recent_values_.push_back(value);

    std::vector<int> counts(30, 0);
    for (auto v : recent_values_) {
        counts[v]++;
    }

    uint8_t most_popular = std::distance(counts.begin(), std::max_element(counts.begin(), counts.end()));
    RCLCPP_INFO(this->get_logger(), "Most popular number in the last 10 reads: %d", most_popular);

    send_to_uart(std::to_string(most_popular));
}

void ImageProcessorNode::send_to_uart(const std::string& data)
{
    if (uart_fd_ != -1) {
        write(uart_fd_, data.c_str(), data.size());
        write(uart_fd_, "\n", 1); // Newline
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
