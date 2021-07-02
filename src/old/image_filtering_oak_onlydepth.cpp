#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class ImageFiltering
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::CameraSubscriber sub_;
    image_transport::CameraPublisher pub_;

    cv::Mat new_image_;
    sensor_msgs::CameraInfo new_info_;

    bool reading_;


public:

    bool ready_;

    ImageFiltering(): it_(nh_)
    {
        reading_ = true;

        sub_ = it_.subscribeCamera("/stereo_rgb_node/stereo/image", 1, &ImageFiltering::callback, this);
        pub_ = it_.advertiseCamera("/stereo_rgb_node/filtered/image", 1);
    }


    void callback(const sensor_msgs::ImageConstPtr& image_raw,
                  const sensor_msgs::CameraInfoConstPtr& camera_info)
    {
        if (reading_)
        {
            new_image_ = (cv_bridge::toCvShare(image_raw)->image).clone();
            new_info_ = *camera_info;
        }
        ready_ = true;
    }


    void mainLoop(int mask_size, double image_scale, double min_range, double max_range, int count)
    {
        reading_ = false;
        cv::Mat input_image = new_image_;
        sensor_msgs::CameraInfo input_info = updateInfo(new_info_, image_scale);
        reading_ = true;

        cv::Mat resized_image; cv::resize(input_image, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST);

        cv::Mat byte_image = convertFloatToByte(resized_image, min_range, max_range);

        cv::Mat blurred_image; cv::medianBlur(byte_image, blurred_image, mask_size);

        cv::Mat float_image = convertByteToFloat(blurred_image, min_range, max_range);

        std_msgs::Header header = input_info.header;
        header.seq = count;

        sensor_msgs::Image output_image;
        cv_bridge::CvImage(header, "32FC1", float_image).toImageMsg(output_image);

        sensor_msgs::CameraInfo output_info = input_info;

        pub_.publish(output_image, output_info);
    }


    sensor_msgs::CameraInfo updateInfo(sensor_msgs::CameraInfo camera_info, float image_scale)
    {
        sensor_msgs::CameraInfo new_info;

        new_info.header = camera_info.header;

        new_info.height = camera_info.height * image_scale;
        new_info.width = camera_info.width * image_scale;

        new_info.distortion_model = camera_info.distortion_model;

        for (int k = 0; k < 8; k++)
            new_info.D.push_back(camera_info.D[k]);

        for (int k = 0; k < 9; k++)
            new_info.K[k] = camera_info.K[k] * image_scale;

        new_info.R = camera_info.R;

        for (int k = 0; k < 12; k++)
            new_info.P[k] = camera_info.P[k] * image_scale;

        return new_info;
    }


    cv::Mat convertFloatToByte(cv::Mat float_image, double min_range, double max_range)
    {
        int h = float_image.rows;
        int w = float_image.cols;
        int numel = h * w;

        double prec = (max_range - min_range) / 255.0;

        cv::Mat byte_image = cv::Mat(h, w, CV_8UC1);

        float* float_ptr = (float*)float_image.data;
        uint8_t* byte_ptr = byte_image.data;

        for (int n = 0; n < numel; n++)
        {
            int value = (int)round((float_ptr[n] - min_range) / prec);
            byte_ptr[n] = value > 0 && value < 255 ? value : 0;
            //ROS_INFO("%f %d %d", float_ptr[n], value, byte_ptr[n]);
        }

        return byte_image;
    }


    cv::Mat convertByteToFloat(cv::Mat byte_image, double min_range, double max_range)
    {
        int height = byte_image.rows;
        int width = byte_image.cols;
        int numel = height * width;

        double prec = (max_range - min_range) / 255.0;

        cv::Mat float_image = cv::Mat(height, width, CV_32FC1);

        uint8_t* byte_ptr = byte_image.data;
        float* float_ptr = (float*)float_image.data;

        for (int n = 0; n < numel; n++)
            float_ptr[n] = byte_ptr[n] > 0 ? prec * byte_ptr[n] + min_range : 0.0;

        return float_image;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_filtering_node");

    ros::NodeHandle nh;

    int mask_size; nh.param("mask_size", mask_size, 5);
    int publish_rate; nh.param("publish_rate", publish_rate, 20);
    double image_scale; nh.param("image_scale", image_scale, 0.5);
    double min_range; nh.param("min_range", min_range, 0.0);
    double max_range; nh.param("max_range", max_range, 10.0);

    ImageFiltering filter;

    filter.ready_ = false;
    while (!filter.ready_ && ros::ok())
        ros::spinOnce();

    int count = 0;

    ros::Rate rate(publish_rate);
    while (ros::ok())
    {
        filter.mainLoop(mask_size, image_scale, min_range, max_range, count++);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
