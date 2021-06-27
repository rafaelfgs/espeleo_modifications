#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

const int BUFFER_SIZE = 15;
const int MASK_SIZE = 15;

class ImageFiltering
{

    public:

        ImageFiltering():

            it(nh)
            {
                first = true;

                //inf_sub = nh.subscribe("/usb_cam/camera_info", BUFFER_SIZE, &ImageFiltering::callback, this);
                //inf_pub = nh.advertise("/republished/camera_info", BUFFER_SIZE);

                img_sub = it.subscribe("/usb_cam/image_raw", BUFFER_SIZE, &ImageFiltering::callback, this);
                img_pub = it.advertise("/republished/image_raw", BUFFER_SIZE);
            }


    private:

        bool first;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;

        image_transport::Subscriber img_sub;
        image_transport::Publisher img_pub;

        cv::Mat buff_pix[BUFFER_SIZE];
        cv::Mat buff_idx[BUFFER_SIZE];


    cv::Mat medianSequence(cv::Mat img)
    {

        uint8_t* img_ptr = img.data;
        uint8_t* pix_ptr[BUFFER_SIZE];
        uint8_t* idx_ptr[BUFFER_SIZE];

        for (int k = 0; k < BUFFER_SIZE; k++)
        {
            pix_ptr[k] = buff_pix[k].data;
            idx_ptr[k] = buff_idx[k].data;
        }

        int numel = img.rows * img.cols;

        for (int n = 0; n < numel; n++)
        {
            int idx_old = 0;
            int idx_new = 0;

            //if(n==100000){ROS_INFO("\n\n");}
            //if(n==100000){ROS_INFO("n=%d img=%d", n, img_ptr[n]);}

            for (int k = 0; k < BUFFER_SIZE; k++)
            {
                if (pix_ptr[k][n] < img_ptr[n])
                    idx_new = k + 1;

                if (idx_ptr[k][n] == 0)
                    idx_old = k;

                //if(n==100000){ROS_INFO("idx=%d pix=%d", idx_ptr[k][n], pix_ptr[k][n]);}

                idx_ptr[k][n]--;
            }

            //if(n==100000){ROS_INFO("old=%d new=%d", idx_old, idx_new);}

            if (idx_old < idx_new)
            {
                idx_new--;
                for (int i = idx_old; i < idx_new - 1; i++)
                    pix_ptr[i][n] = pix_ptr[i + 1][n];
            }
            else
            {
                for (int i = idx_old; i >= idx_new + 1; i--)
                    pix_ptr[i][n] = pix_ptr[i - 1][n];
            }

            //if(n==100000){ROS_INFO("\n");}
            //for (int k = 0; k < BUFFER_SIZE; k++)
            //    if(n==100000){ROS_INFO("pix=%d idx=%d", pix_ptr[k][n], idx_ptr[k][n]);}

            pix_ptr[idx_new][n] = img_ptr[n];
            idx_ptr[idx_old][n] = BUFFER_SIZE - 1;

            //if(n==100000){ROS_INFO("\n");}
            //for (int k = 0; k < BUFFER_SIZE; k++)
            //    if(n==100000){ROS_INFO("pix=%d idx=%d", pix_ptr[k][n], idx_ptr[k][n]);}
        }

        return buff_pix[BUFFER_SIZE / 2];
    }


    void callback(const sensor_msgs::ImageConstPtr& msg_sub)
    {
        //ROS_INFO("%d", 0);

        cv::Mat img_raw = cv_bridge::toCvShare(msg_sub, "mono8")->image;

        if (first)
        {
            first = false;

            for (int k = 0; k < BUFFER_SIZE; k++)
            {
                buff_pix[k] = img_raw.clone();
                buff_idx[k] = cv::Mat(img_raw.rows, img_raw.cols, CV_8UC1, k);
            }
        }

        cv::Mat filt_seq = medianSequence(img_raw);

        cv::Mat filt_blur; cv::medianBlur(filt_seq, filt_blur, MASK_SIZE);

        sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(msg_sub->header, "mono8", filt_blur).toImageMsg();
        img_pub.publish(msg_pub);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_filtering_node");
    ImageFiltering filter;

    ros::spin();
    return 0;
}
