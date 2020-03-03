#include <ros/ros.h>
#include <Eigen/Core>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp> 


#include <queue>
#include <unistd.h>

using namespace cv;
int main(int argc, char** argv){
    ros::init(argc, argv, "pub_rs2_image");

    ros::NodeHandle nh;

    ros::Publisher pub_depth     = nh.advertise<cv_bridge::CvImage>("/depth", 100);
    ros::Publisher pub_ir0       = nh.advertise<cv_bridge::CvImage>("/image0",100);
    ros::Publisher pub_ir1       = nh.advertise<cv_bridge::CvImage>("/image1",100);
    ros::Publisher pub_rgb       = nh.advertise<cv_bridge::CvImage>("/rgb",100);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    int w = 640, h = 480;
    int frequency = 30;

    //enable ir, depth and rgb
    cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, frequency);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, w, h, RS2_FORMAT_Y8, frequency);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, w, h, RS2_FORMAT_Y8, frequency);
    cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, frequency);

    // Start streaming with default recommended configuration
    pipe.start(cfg);

    bool end_loop = false;
    int warm_up = 30;

    cv_bridge::CvImage depth, image0, image1, rgb;
    //check http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html to see what encoding you can use
    depth.encoding     = "16UC1";
    image0.encoding    = "8UC1";
    image1.encoding    = "8UC1";
    rgb.encoding       = "8UC3";

    int counter = 0;

    ROS_INFO("start publishing..............");

    while(ros::ok()){
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        if(warm_up>0){
            // Camera warmup - dropping several first frames to let auto-exposure stabilize
            warm_up--;
            continue;
        }

        // ros::Time time_start = ros::Time::now();
        rs2::depth_frame depth_frame = data.get_depth_frame();
        rs2::video_frame color = data.get_color_frame();
        rs2::video_frame infra0 = data.get_infrared_frame(1);
        rs2::video_frame infra1 = data.get_infrared_frame(2);

        // Create OpenCV matrix of size (w,h) from depth rgb and ir stream
        Mat depth_image(Size(w, h), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        Mat color_image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        Mat ir0(Size(w, h), CV_8UC1, (void*)infra0.get_data(), Mat::AUTO_STEP);
        Mat ir1(Size(w, h), CV_8UC1, (void*)infra1.get_data(), Mat::AUTO_STEP);
        
        //assign value to cv image
        std_msgs::Header header;
        header.frame_id  = counter;
        header.stamp  = ros::Time::now();

        depth.header     = header;
        rgb.header       = header;
        image0.header    = header;
        image1.header    = header;
        
        depth.image      = depth_image;
        rgb.image        = color_image;
        image0.image     = ir0;
        image1.image     = ir1;
        
        pub_depth.publish(depth);
        pub_ir0.publish(image0);
        pub_ir1.publish(image1);
        pub_rgb.publish(rgb);

        // ros::Time time_end = ros::Time::now();
        // ros::Duration time_spend = time_end - time_start;
        // ROS_INFO("we spend %f seconds to get and publish image", time_spend.toSec());

        counter++ ;
    }
}

