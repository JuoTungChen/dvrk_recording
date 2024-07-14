#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Global variables to store the latest images
cv::Mat global_image_left;
cv::Mat global_image_right;
bool new_image_left = false;
bool new_image_right = false;

class ImageProcessor
{
public:
    ImageProcessor()
    {
        image_transport::ImageTransport it(nh);
        sub_left = it.subscribe("/jhu_daVinci/left/image_raw", 1, &ImageProcessor::imageCallbackLeft, this);
        sub_right = it.subscribe("/jhu_daVinci/right/image_raw", 1, &ImageProcessor::imageCallbackRight, this);
    }

    void imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            global_image_left = cv_bridge::toCvCopy(msg, "bgr8")->image;
            new_image_left = true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            global_image_right = cv_bridge::toCvCopy(msg, "bgr8")->image;
            new_image_right = true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void processImage(cv::Mat &image, bool draw_rectangle)
    {
        // cv::resize(image, image, cv::Size(960, 1080)); // Adjusted from Python's dynamic calculation
        
        if (draw_rectangle)
        {
            cv::rectangle(image, cv::Point(900, 930), cv::Point(1500, 1330), cv::Scalar(255, 0, 0), 3);
            cv::rectangle(image, cv::Point(100, 930), cv::Point(300, 1330), cv::Scalar(0, 255, 0), 3);
        }
    }

private:
    ros::NodeHandle nh;
    image_transport::Subscriber sub_left;
    image_transport::Subscriber sub_right;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_img");
    ImageProcessor processor;
    ros::Rate loop_rate(5);  // Loop at 5 Hz

    while (ros::ok()) {
        if (new_image_left) {
            processor.processImage(global_image_left, true);
            cv::imshow("Frame Left", global_image_left);
            new_image_left = false;
        }
        if (new_image_right) {
            processor.processImage(global_image_right, false);
            cv::imshow("Frame Right", global_image_right);
            new_image_right = false;
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {  // 'q' or ESC key
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyAllWindows();
    return 0;
}
