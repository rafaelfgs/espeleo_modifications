#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


const int BUFFER_SIZE = 7;


class ImageFiltering
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::CameraSubscriber sub_;
    image_transport::CameraPublisher pub_;

    cv::Mat new_image_;
    sensor_msgs::CameraInfo new_info_;

    cv::Mat buffer_pixel[BUFFER_SIZE];
    cv::Mat buffer_index[BUFFER_SIZE];

    bool reading_, first_;


public:

    bool ready_;

    ImageFiltering(): it_(nh_)
    {
        first_ = true;
        reading_ = true;

        sub_ = it_.subscribeCamera("/stereo_rgb_node/stereo/image", BUFFER_SIZE, &ImageFiltering::callback, this);
        pub_ = it_.advertiseCamera("/stereo_rgb_node/filtered/image", BUFFER_SIZE);
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


    void mainLoop(int mask_size, double image_scale, double min_range, double max_range)
    {
        reading_ = false;
        cv::Mat input_image = new_image_;
        sensor_msgs::CameraInfo input_info = new_info_;
        reading_ = true;

        cv::Mat resized_image; cv::resize(input_image, resized_image, cv::Size(), image_scale, image_scale, cv::INTER_NEAREST);

        cv::Mat byte_image = convertFloatToByte(resized_image, min_range, max_range);

        if (first_)
        {
            first_ = false;

            for (int k = 0; k < BUFFER_SIZE; k++)
            {
                buffer_pixel[k] = byte_image.clone();
                buffer_index[k] = cv::Mat(byte_image.rows, byte_image.cols, CV_8UC1, k);
            }
        }

        cv::Mat sequenced_image = medianSequence(byte_image);

        cv::Mat blurred_image; cv::medianBlur(sequenced_image, blurred_image, mask_size);

        cv::Mat float_image = convertByteToFloat(blurred_image, min_range, max_range);

        sensor_msgs::Image output_image;
        cv_bridge::CvImage(input_info.header, "32FC1", float_image).toImageMsg(output_image);
        sensor_msgs::CameraInfo output_info = input_info;

        pub_.publish(output_image, output_info);
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


    cv::Mat medianSequence(cv::Mat image)
    {
        int height = image.rows;
        int width = image.cols;
        int numel = height * width;

        uint8_t* image_ptr = image.data;
        uint8_t* pixel_ptr[BUFFER_SIZE];
        uint8_t* index_ptr[BUFFER_SIZE];

        for (int k = 0; k < BUFFER_SIZE; k++)
        {
            pixel_ptr[k] = buffer_pixel[k].data;
            index_ptr[k] = buffer_index[k].data;
        }

        for (int n = 0; n < numel; n++)
        {
            int idx_old = 0;
            int idx_new = 0;

            //if(n==100000){ROS_INFO("\n\n");}
            //if(n==100000){ROS_INFO("n=%d img=%d", n, image_ptr[n]);}

            for (int k = 0; k < BUFFER_SIZE; k++)
            {
                //if(n==100000){ROS_INFO("%d", image_ptr[n]);}
                //if(n==100000){ROS_INFO("%d", pixel_ptr[k][n]);}
                if (pixel_ptr[k][n] < image_ptr[n])
                    idx_new = k + 1;

                if (index_ptr[k][n] == 0)
                    idx_old = k;

                //if(n==100000){ROS_INFO("idx=%d pix=%d", index_ptr[k][n], pixel_ptr[k][n]);}

                index_ptr[k][n]--;
            }

            //if(n==100000){ROS_INFO("old=%d new=%d", idx_old, idx_new);}

            if (idx_old < idx_new)
            {
                idx_new--;
                for (int i = idx_old; i < idx_new - 1; i++)
                    pixel_ptr[i][n] = pixel_ptr[i + 1][n];
            }
            else
            {
                for (int i = idx_old; i >= idx_new + 1; i--)
                    pixel_ptr[i][n] = pixel_ptr[i - 1][n];
            }

            //if(n==100000){ROS_INFO("\n");}
            //for (int k = 0; k < BUFFER_SIZE; k++)
                //if(n==100000){ROS_INFO("pix=%d idx=%d", pixel_ptr[k][n], index_ptr[k][n]);}

            pixel_ptr[idx_new][n] = image_ptr[n];
            index_ptr[idx_old][n] = BUFFER_SIZE - 1;

            //if(n==100000){ROS_INFO("\n");}
            //for (int k = 0; k < BUFFER_SIZE; k++)
                //if(n==100000){ROS_INFO("pix=%d idx=%d", pixel_ptr[k][n], index_ptr[k][n]);}
        }

        return buffer_pixel[BUFFER_SIZE / 2];
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

    ros::Rate rate(publish_rate);
    while (ros::ok())
    {
        filter.mainLoop(mask_size, image_scale, min_range, max_range);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();

    return 0;
}
