// // #include <ros/ros.h>
// // #include <sensor_msgs/Image.h>
// // #include <cv_bridge/cv_bridge.h>
// // #include <opencv2/opencv.hpp>
// // #include <iostream>

// // class ImageSubscriber {
// // private:
// //     ros::NodeHandle nh_;
// //     ros::Subscriber image_sub_;
    
// // public:
// //     ImageSubscriber() {
// //         // Subscribe to camera topic as shown in the slides
// //         image_sub_ = nh_.subscribe("/camera/image_raw", 1, &ImageSubscriber::imageCallback, this);
// //         ROS_INFO("Image subscriber initialized");
// //     }
    
// //     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
// //         try {
// //             // Convert ROS image to OpenCV format using cv_bridge
// //             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
// //             // Process the image - here we're just displaying it
// //             cv::imshow("Image Window", cv_ptr->image);
// //             cv::waitKey(3);
            
// //             // Log that we received an image
// //             ROS_INFO("Received image: %dx%d", cv_ptr->image.cols, cv_ptr->image.rows);
// //         }
// //         catch (cv_bridge::Exception& e) {
// //             ROS_ERROR("cv_bridge exception: %s", e.what());
// //         }
// //     }
// // };

// // int main(int argc, char** argv) {
// //     ros::init(argc, argv, "image_subscriber");
    
// //     ImageSubscriber imageSubscriber;
    
// //     ros::spin();
    
// //     return 0;
// // }

// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>

// class ImageSubscriber {
// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber image_sub_;

// public:
//     ImageSubscriber() {
//         image_sub_ = nh_.subscribe("/camera/image", 1, &ImageSubscriber::imageCallback, this);
//         ROS_INFO("Image subscriber initialized");
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         try {
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat image = cv_ptr->image;

//             std::vector<cv::Mat> channels;
//             cv::split(image, channels); // channels[0]=B, channels[1]=G, channels[2]=R

//             cv::Mat gray;
//             cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

//             cv::imshow("Original", image);
//             cv::imshow("Red", channels[2]);
//             cv::imshow("Green", channels[1]);
//             cv::imshow("Blue", channels[0]);
//             cv::imshow("Grayscale", gray);

//             cv::waitKey(1);
//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "image_subscriber");
//     ImageSubscriber img_sub;
//     ros::spin();
//     return 0;
// }

// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <image_transport/image_transport.h>

// class ImageSubscriber {
// private:
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;

// public:
//     ImageSubscriber() : it_(nh_) {
//         // Subscribe to the camera topic (remapped in the launch file)
//         image_sub_ = it_.subscribe("/camera/image", 1, &ImageSubscriber::imageCallback, this);
//         ROS_INFO("Image subscriber initialized - displaying R,G,B channels and grayscale");
        
//         // Create windows for displaying images
//         cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
//         cv::namedWindow("Red Channel", cv::WINDOW_AUTOSIZE);
//         cv::namedWindow("Green Channel", cv::WINDOW_AUTOSIZE);
//         cv::namedWindow("Blue Channel", cv::WINDOW_AUTOSIZE);
//         cv::namedWindow("Grayscale", cv::WINDOW_AUTOSIZE);
//     }

//     ~ImageSubscriber() {
//         // Destroy all windows when the node is shut down
//         cv::destroyAllWindows();
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         try {
//             // Convert ROS image to OpenCV format
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat image = cv_ptr->image;

//             // Split the image into B, G, R channels
//             std::vector<cv::Mat> bgr_channels;
//             cv::split(image, bgr_channels);
            
//             // Create single-channel images with zeros
//             cv::Mat r_channel = cv::Mat::zeros(image.size(), CV_8UC3);
//             cv::Mat g_channel = cv::Mat::zeros(image.size(), CV_8UC3);
//             cv::Mat b_channel = cv::Mat::zeros(image.size(), CV_8UC3);
            
//             // Put each channel in its proper place
//             std::vector<cv::Mat> r_channels = {cv::Mat::zeros(image.size(), CV_8UC1), 
//                                               cv::Mat::zeros(image.size(), CV_8UC1), 
//                                               bgr_channels[2]};
//             cv::merge(r_channels, r_channel);
            
//             std::vector<cv::Mat> g_channels = {cv::Mat::zeros(image.size(), CV_8UC1), 
//                                               bgr_channels[1], 
//                                               cv::Mat::zeros(image.size(), CV_8UC1)};
//             cv::merge(g_channels, g_channel);
            
//             std::vector<cv::Mat> b_channels = {bgr_channels[0], 
//                                               cv::Mat::zeros(image.size(), CV_8UC1), 
//                                               cv::Mat::zeros(image.size(), CV_8UC1)};
//             cv::merge(b_channels, b_channel);
            
//             // Convert to grayscale
//             cv::Mat gray;
//             cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            
//             // Convert grayscale to 3-channel for visualization
//             cv::Mat gray_display;
//             cv::cvtColor(gray, gray_display, cv::COLOR_GRAY2BGR);

//             // Display the images
//             cv::imshow("Original", image);
//             cv::imshow("Red Channel", r_channel);
//             cv::imshow("Green Channel", g_channel);
//             cv::imshow("Blue Channel", b_channel);
//             cv::imshow("Grayscale", gray_display);

//             cv::waitKey(1); // Update the windows
            
//             ROS_INFO_THROTTLE(5, "Processing image: %dx%d", image.cols, image.rows);
//         } 
//         catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "image_subscriber");
    
//     ImageSubscriber img_sub;
    
//     ros::spin();
    
//     return 0;
// }

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

class ImageSubscriber {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

public:
    ImageSubscriber() : it_(nh_) {
        // Subscribe to the camera topic (remapped in the launch file)
        image_sub_ = it_.subscribe("/camera/image", 1, &ImageSubscriber::imageCallback, this);
        ROS_INFO("Image subscriber initialized - displaying all channels in one window");
        
        // Create a single window for displaying all images
        cv::namedWindow("All Channels", cv::WINDOW_AUTOSIZE);
    }

    ~ImageSubscriber() {
        // Destroy all windows when the node is shut down
        cv::destroyAllWindows();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            
            // Get image dimensions
            int height = image.rows;
            int width = image.cols;
            
            // Create a large canvas to hold all images (3x2 grid)
            cv::Mat canvas = cv::Mat::zeros(height * 2, width * 3, CV_8UC3);
            
            // Split the image into B, G, R channels
            std::vector<cv::Mat> bgr_channels;
            cv::split(image, bgr_channels);
            
            // Create single-channel images with zeros
            cv::Mat r_channel = cv::Mat::zeros(image.size(), CV_8UC3);
            cv::Mat g_channel = cv::Mat::zeros(image.size(), CV_8UC3);
            cv::Mat b_channel = cv::Mat::zeros(image.size(), CV_8UC3);
            
            // Put each channel in its proper place
            std::vector<cv::Mat> r_channels = {cv::Mat::zeros(image.size(), CV_8UC1), 
                                              cv::Mat::zeros(image.size(), CV_8UC1), 
                                              bgr_channels[2]};
            cv::merge(r_channels, r_channel);
            
            std::vector<cv::Mat> g_channels = {cv::Mat::zeros(image.size(), CV_8UC1), 
                                              bgr_channels[1], 
                                              cv::Mat::zeros(image.size(), CV_8UC1)};
            cv::merge(g_channels, g_channel);
            
            std::vector<cv::Mat> b_channels = {bgr_channels[0], 
                                              cv::Mat::zeros(image.size(), CV_8UC1), 
                                              cv::Mat::zeros(image.size(), CV_8UC1)};
            cv::merge(b_channels, b_channel);
            
            // Convert to grayscale
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            
            // Convert grayscale to 3-channel for visualization
            cv::Mat gray_display;
            cv::cvtColor(gray, gray_display, cv::COLOR_GRAY2BGR);
            
            // Copy all images to the canvas in a 3x2 grid
            // First row: RGB, R, G
            // Second row: B, Gray
            image.copyTo(canvas(cv::Rect(0, 0, width, height)));                      // Original (top-left)
            r_channel.copyTo(canvas(cv::Rect(width, 0, width, height)));              // Red (top-center)
            g_channel.copyTo(canvas(cv::Rect(width*2, 0, width, height)));            // Green (top-right)
            b_channel.copyTo(canvas(cv::Rect(0, height, width, height)));             // Blue (bottom-left)
            gray_display.copyTo(canvas(cv::Rect(width, height, width, height)));      // Gray (bottom-center)
            
            // Add labels to each section
            cv::putText(canvas, "RGB", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            cv::putText(canvas, "RED", cv::Point(width + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            cv::putText(canvas, "GREEN", cv::Point(width*2 + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::putText(canvas, "BLUE", cv::Point(10, height + 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 2);
            cv::putText(canvas, "GRAY", cv::Point(width + 10, height + 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 200, 200), 2);

            // Display the combined image
            cv::imshow("All Channels", canvas);
            cv::waitKey(1); // Update the window
            
            ROS_INFO_THROTTLE(5, "Processing image: %dx%d", image.cols, image.rows);
        } 
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_subscriber");
    
    ImageSubscriber img_sub;
    
    ros::spin();
    
    return 0;
}