#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

int BUFFER_SIZE;
int MASK_SIZE;

cv::Mat IMAGE;
sensor_msgs::CameraInfo CINFO;

/*
class ImageFiltering
{

    public:

        ImageFiltering():

            it(nh)
            {
                first = true;

                //inf_sub = nh.subscribe("/usb_cam/camera_info", BUFFER_SIZE, &ImageFiltering::callback, this);
                //inf_pub = nh.advertise("/republished/camera_info", BUFFER_SIZE);

                img_sub = it.subscribe("/stereo_rgb_node/stereo/image", BUFFER_SIZE, &ImageFiltering::callback, this);
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


    cv::Mat preProcessing(cv::Mat img_raw)
    {
        int h = img_raw.rows;
        int w = img_raw.cols;

        cv::Mat img_proc = cv::Mat(h, w, CV_32FC1, 0.0);

        for (int i = 0; i < h; i += 2)
        {
            float* raw_ptr = img_raw.ptr<float>(i);
            float* proc_ptr = img_proc.ptr<float>(i);

            for (int j = 0; j < w; j += 2)
            {
                if (raw_ptr[j] > 0)
                    proc_ptr[j] = raw_ptr[j];
                //ROS_INFO("raw = %f, proc = %f", raw_ptr[j], proc_ptr[j]);
            }
        }

        return img_proc;
    }


    cv::Mat medianSequence(cv::Mat img)
    {
        float* img_ptr = (float*)img.data;
        float* pix_ptr[BUFFER_SIZE];
        uint8_t* idx_ptr[BUFFER_SIZE];

        for (int k = 0; k < BUFFER_SIZE; k++)
        {
            pix_ptr[k] = (float*)buff_pix[k].data;
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

        cv::Mat img_raw = cv_bridge::toCvShare(msg_sub)->image;
        //ROS_INFO("%f", img_raw.at<float>(220, 220));
        //cv::Mat img_proc = preProcessing(img_raw);
        cv::Mat img_proc; cv::resize(img_raw, img_proc, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        //ROS_INFO("%f", img_proc.at<float>(220, 220));

        if (first)
        {
            first = false;

            for (int k = 0; k < BUFFER_SIZE; k++)
            {
                buff_pix[k] = img_proc.clone();
                buff_idx[k] = cv::Mat(img_proc.rows, img_proc.cols, CV_8UC1, k);
            }
        }

        cv::Mat filt_seq = medianSequence(img_proc);
        //cv::Mat filt_seq = img_proc.clone();

        cv::Mat filt_blur; cv::medianBlur(filt_seq, filt_blur, MASK_SIZE);

        sensor_msgs::ImagePtr msg_pub = cv_bridge::CvImage(msg_sub->header, "32FC1", filt_blur).toImageMsg();
        img_pub.publish(msg_pub);
    }

};
*/


void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cinfo)
{
    IMAGE = cv_bridge::toCvShare(image)->image;
    ROS_INFO("%f", IMAGE.at<float>(220, 220));
    CINFO = *cinfo;
    ROS_INFO("%f", CINFO.D[0]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_filtering_node");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    nh.param("buffer_size", BUFFER_SIZE, 7);
    nh.param("mask_size", MASK_SIZE, 5);

    cv::Mat buff_pix[BUFFER_SIZE];
    cv::Mat buff_idx[BUFFER_SIZE];

    image_transport::CameraSubscriber sub = it.subscribeCamera("stereo_rgb_node/stereo/image", BUFFER_SIZE, callback);

    image_transport::CameraPublisher pub = it.advertiseCamera("/stereo_rgb_node/filtered/image", BUFFER_SIZE);

    while (ros::ok() && IMAGE.empty()) { }

    sensor_msgs::Image msg_image;
    sensor_msgs::CameraInfo msg_cinfo;

    ros::Rate rate(2);

    while (ros::ok())
    {

        ROS_INFO("%d", 0);
        cv_bridge::CvImage(CINFO.header, "32FC1", IMAGE).toImageMsg(msg_image);
        msg_cinfo = CINFO;

        pub.publish(msg_image, msg_cinfo);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
