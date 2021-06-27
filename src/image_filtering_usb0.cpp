#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

const int BUFFER_SIZE = 10;
const int MASK_SIZE = 2;

class ImageFiltering
{

    public:

        ImageFiltering():
            it_(nh_)
            {
                k_ = 0;
                sub_ = it_.subscribe("/usb_cam/image_raw", BUFFER_SIZE, &ImageFiltering::callback, this);
                pub_ = it_.advertise("/republished/image_raw", BUFFER_SIZE);
            }


    private:

        int k_;
        cv::Mat buffer_[BUFFER_SIZE];
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber sub_;
        image_transport::Publisher pub_;


    cv::Mat filter(cv::Mat buffer[BUFFER_SIZE])
    {
        int h = buffer[0].rows;
        int w = buffer[0].cols;

        int hm = h - MASK_SIZE;
        int wm = w - MASK_SIZE;

        int numel = (2 * MASK_SIZE + 1) * (2 * MASK_SIZE + 1) * BUFFER_SIZE; 
        int pixel[numel];
        int* pixel_ptr = pixel;

        uint8_t* buffer_ptr[BUFFER_SIZE];
        for (int k = 0; k < BUFFER_SIZE; k++)
            buffer_ptr[k] = buffer[k].data;

        ROS_INFO("%d", buffer_ptr[0][0]);

        cv::Mat filtered(h, w, CV_8UC1);
        uint8_t* filtered_ptr = filtered.data;

        for (int i = 0; i < MASK_SIZE; i++)
            for (int j = 0; j < w; j++)
                filtered_ptr[i * w + j] = 0;

        for (int i = MASK_SIZE; i < hm; i++)
            for (int j = 0; j < MASK_SIZE; j++)
                filtered_ptr[i * w + j] = 0;

        for (int i = MASK_SIZE; i < hm; i++)
            for (int j = wm; j < w; j++)
                filtered_ptr[i * w + j] = 0;

        for (int i = hm; i < h; i++)
            for (int j = 0; j < w; j++)
                filtered_ptr[i * w + j] = 0;
        

        for (int i = MASK_SIZE - 1; i < hm; i++)
        {
            for (int j = MASK_SIZE - 1; j < wm; j++)
            {
                //for (int k = 0; k < BUFFER_SIZE; k++)
                //{
                    //for (int n = 0; n < numel; n++)
                    //{
                        pixel_ptr[0] = buffer_ptr[0][i * w + j];
                    //}
                //}
                filtered_ptr[i * w + j] = pixel_ptr[0];
            }
        }
        return filtered;
    }


    void callback(const sensor_msgs::ImageConstPtr& msg_sub)
    {
        buffer_[k_++ % BUFFER_SIZE] = cv_bridge::toCvShare(msg_sub, "mono8")->image;

        if (k_ > 9)
        {
            cv::Mat filtered = filter(buffer_);
            sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered).toImageMsg();
            pub_.publish(msg_pub);
        }
    }

};

int main(int argc, char** argv)
{
    ros::init( argc, argv, "image_filtering_node" );
    ImageFiltering filter;

    ros::spin();
    return 0;
}
