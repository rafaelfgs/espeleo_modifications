#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

const float IMAGE_SCALE = 0.5;
const int MASK_SIZE = 5;


class ImageFiltering
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    //nh_.param("image_scale", IMAGE_SCALE, 0.5);
    //nh_.param("mask_size", MASK_SIZE, 5);

    image_transport::CameraSubscriber sub_;
    image_transport::CameraPublisher pub_;


public:

    ImageFiltering(): it_(nh_)
    {
        sub_ = it_.subscribeCamera("/stereo_rgb_node/stereo/image", 1, &ImageFiltering::callback, this);
        pub_ = it_.advertiseCamera("/stereo_rgb_node/filtered/image", 1);
    }


    void callback(const sensor_msgs::ImageConstPtr& input_image_msg,
                  const sensor_msgs::CameraInfoConstPtr& input_info_msg)
    {
        //ROS_INFO("%d", 0);
        cv::Mat input_image = cv_bridge::toCvShare(input_image_msg)->image;
        sensor_msgs::CameraInfo input_info = *input_info_msg;

        //ROS_INFO("%d", 1);
        cv::Mat resized_image;
        cv::resize(input_image, resized_image, cv::Size(), IMAGE_SCALE, IMAGE_SCALE, cv::INTER_NEAREST);

        //ROS_INFO("%d", 2);
        cv::Mat blurred_image;
        cv::medianBlur(resized_image, blurred_image, MASK_SIZE);

        //ROS_INFO("%d", 4);
        sensor_msgs::Image output_image_msg;
        cv_bridge::CvImage(input_info.header, "32FC1", blurred_image).toImageMsg(output_image_msg);
        sensor_msgs::CameraInfo output_info_msg = input_info;

        pub_.publish(output_image_msg, output_info_msg);
    }

};  


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_filtering_node");
    ImageFiltering filter;
    ros::spin();
    return 0;
}
