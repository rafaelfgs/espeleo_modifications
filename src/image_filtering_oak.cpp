#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class ImageFiltering
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::CameraSubscriber color_sub_;
    image_transport::CameraSubscriber depth_sub_;

    image_transport::CameraPublisher color_pub_;
    image_transport::CameraPublisher depth_pub_;

    cv::Mat new_color_image_;
    cv::Mat new_depth_image_;

    sensor_msgs::CameraInfo new_color_info_;
    sensor_msgs::CameraInfo new_depth_info_;

    bool reading_;


public:

    bool color_ready_, depth_ready_;

    ImageFiltering(): it_(nh_)
    {
        reading_ = true;

        color_sub_ = it_.subscribeCamera("/stereo_rgb_node/color/image", 10, &ImageFiltering::colorCallback, this);
        depth_sub_ = it_.subscribeCamera("/stereo_rgb_node/stereo/image", 10, &ImageFiltering::depthCallback, this);

        color_pub_ = it_.advertiseCamera("/stereo_rgb_node/filtered/color/image", 10);
        depth_pub_ = it_.advertiseCamera("/stereo_rgb_node/filtered/depth/image", 10);
    }


    void colorCallback(const sensor_msgs::ImageConstPtr& image,
                       const sensor_msgs::CameraInfoConstPtr& info)
    {
        if (reading_)
        {
            new_color_image_ = (cv_bridge::toCvShare(image)->image).clone();
            new_color_info_ = *info;
        }
        color_ready_ = true;
    }


    void depthCallback(const sensor_msgs::ImageConstPtr& image,
                       const sensor_msgs::CameraInfoConstPtr& info)
    {
        if (reading_)
        {
            new_depth_image_ = (cv_bridge::toCvShare(image)->image).clone();
            new_depth_info_ = *info;
        }
        depth_ready_ = true;
    }


    void mainLoop(int mask_size, double image_scale, double min_range, double max_range, int count)
    {
        struct timeval to, tf;
        gettimeofday(&to, NULL);

        reading_ = false;

        cv::Mat input_color_image = new_color_image_;
        cv::Mat input_depth_image = new_depth_image_;

        sensor_msgs::CameraInfo input_color_info = new_color_info_;
        sensor_msgs::CameraInfo input_depth_info = new_depth_info_;

        reading_ = true;

        cv::Mat resized_color;
        cv::resize(input_color_image, resized_color, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST);

        cv::Mat resized_depth;
        cv::resize(input_depth_image, resized_depth, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST);

        cv::Mat byte_depth = convertFloatToByte(resized_depth, min_range, max_range);

        //cv::Mat mean_depth; cv::blur(byte_depth, mean_depth, cv::Size(mask_size, mask_size));
        //cv::Mat zeros_depth = mergeZeros(mean_depth, byte_depth);

        cv::Mat median_depth; cv::medianBlur(byte_depth, median_depth, mask_size);
        cv::Mat float_depth = convertByteToFloat(median_depth, min_range, max_range);

        std_msgs::Header header;
        header.seq = count;
        header.stamp = ros::Time::now();

        header.frame_id = "oak_filtered_frame";

        sensor_msgs::Image output_color_image;
        cv_bridge::CvImage(header, "bgr8", resized_color).toImageMsg(output_color_image);

        sensor_msgs::Image output_depth_image;
        cv_bridge::CvImage(header, "32FC1", float_depth).toImageMsg(output_depth_image);

        sensor_msgs::CameraInfo output_color_info = updateInfo(input_color_info, image_scale, count);
        sensor_msgs::CameraInfo output_depth_info = updateInfo(input_depth_info, image_scale, count);

        color_pub_.publish(output_color_image, output_color_info);
        depth_pub_.publish(output_depth_image, output_depth_info);

        gettimeofday(&tf, NULL);
        int t = (1000*tf.tv_sec + tf.tv_usec/1000) - (1000*to.tv_sec + to.tv_usec/1000);
        ROS_INFO("mask_size: %d - %d ms", mask_size, t);
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


    cv::Mat mergeZeros(cv::Mat curr_image, cv::Mat ref_image)
    {
        int height = curr_image.rows;
        int width = curr_image.cols;
        int numel = height * width;

        cv::Mat zeros_image = cv::Mat(height, width, CV_8UC1);

        uint8_t* curr_ptr = curr_image.data;
        uint8_t* ref_ptr = ref_image.data;
        uint8_t* zeros_ptr = zeros_image.data;

        for (int n = 0; n < numel; n++)
            zeros_ptr[n] = ref_ptr[n] > 0 ? curr_ptr[n] : 0;

        return zeros_image;
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


    sensor_msgs::CameraInfo updateInfo(sensor_msgs::CameraInfo camera_info, float image_scale, int count)
    {
        sensor_msgs::CameraInfo new_info;

        new_info.header.seq = count;
        new_info.header.stamp = ros::Time::now();
        new_info.header.frame_id = camera_info.header.frame_id;

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

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_filtering_node");

    ros::NodeHandle nh("~");

    int mask_size; nh.param("mask_size", mask_size, 5);
    int publish_rate; nh.param("publish_rate", publish_rate, 10);
    double image_scale; nh.param("image_scale", image_scale, 0.5);
    double min_range; nh.param("min_range", min_range, 0.0);
    double max_range; nh.param("max_range", max_range, 10.0);

    ImageFiltering filter;

    filter.color_ready_ = false;
    filter.depth_ready_ = false;
    while ((!filter.color_ready_ || !filter.depth_ready_) && ros::ok())
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
