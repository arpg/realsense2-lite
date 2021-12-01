#include <ros/ros.h>
#include <Eigen/Core>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <librealsense2/rs.hpp> 

#include "sensor_msgs/CameraInfo.h"


#include <queue>
#include <unistd.h>

void set_cam_info_default(sensor_msgs::CameraInfo& ci, const rs2_intrinsics& in, const rs2_extrinsics ex);
void set_cam_info(sensor_msgs::CameraInfo& cam, ros::NodeHandle& nh, int cam_id);

double depth_factor = 0.001;

int main(int argc, char** argv){
    ros::init(argc, argv, "reconsruction");

    ros::NodeHandle nh("~");

    bool pub_aligned_rgb = false;
    if(nh.getParam("publish_aligned_rgb", pub_aligned_rgb)){
        ROS_INFO("Publish aligned rgb image? %d", pub_aligned_rgb);
        ROS_INFO("If true, it will slow down the publication speed. Max around 30 fps, 640X480");
    }
    else{
        ROS_INFO("Default not publishng rgb aligned image ");
    }

    ros::Publisher pub_start_imu_stream = nh.advertise<std_msgs::Bool>("imu_start_flag", 10);

    ros::Publisher pub_depth     = nh.advertise<cv_bridge::CvImage>("/depth", 100);
    ros::Publisher pub_ir0       = nh.advertise<cv_bridge::CvImage>("/gray_image0",100);
    ros::Publisher pub_ir1       = nh.advertise<cv_bridge::CvImage>("/gray_image1",100);
    ros::Publisher pub_rgb       = nh.advertise<cv_bridge::CvImage>("/rgb_raw",100);
    ros::Publisher pub_rgb_align = nh.advertise<cv_bridge::CvImage>("/rgb",100);
    ros::Publisher pub_cam_info0  = nh.advertise<sensor_msgs::CameraInfo>("/cam_info0",100);
    ros::Publisher pub_cam_info1  = nh.advertise<sensor_msgs::CameraInfo>("/cam_info1",100);

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    int w  = 640, h = 480;//int w  = 640, h = 480;
    int frequency = 30;

    //cannot achieve more than 15 hz. Image alignment (rgb to depth)
    cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, frequency);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, w, h, RS2_FORMAT_Y8, frequency);//left infrared
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, w, h, RS2_FORMAT_Y8, frequency);//right infrared
    cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_RGB8, frequency);//RS2_FORMAT_BGR8

    // Start streaming with default recommended configuration
    pipe.start(cfg);


    //set camera info
    sensor_msgs::CameraInfo ci0, ci1;
    set_cam_info(ci0, nh, 0);
    set_cam_info(ci1, nh, 1);

    //align color image to depth. See my realsense C++ test example 'depth_align_rgb.cpp', align image will cost about 5~10 ms
    
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::hole_filling_filter  hole_filter;
    hole_filter.set_option(RS2_OPTION_HOLES_FILL,2);
    //rs2::align align_to_img0(RS2_STREAM_INFRARED);

    bool end_loop = false;
    int warm_up = 30;

    using namespace cv;


    std_msgs::Bool start_imu_stream;
    start_imu_stream.data = false;

    cv_bridge::CvImage depth, image0, image1, rgb, rgb_align;
    //check http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html to see what encoding you can use
    depth.encoding     = "32FC1";
    image0.encoding    = "mono8";
    image1.encoding    = "mono8";
    rgb.encoding       = "rgb8";
    rgb_align.encoding = "rgb8";

    int counter = 0;

    ROS_INFO("start publishing..............");

    while(ros::ok() && counter < 3000){
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        if(warm_up>0){
            // Camera warmup - dropping several first frames to let auto-exposure stabilize
            warm_up--;
            continue;
        }

        if( (warm_up == 0) && (!start_imu_stream.data)){
            sleep(0.1);
            start_imu_stream.data = true;
            pub_start_imu_stream.publish(start_imu_stream);
            sleep(0.1);
        }
        
        if(pub_aligned_rgb){
            rs2::frameset frameset = align_to_depth.process(data);
            rs2::video_frame color_align_to_depth = frameset.get_color_frame();
            Mat color_image_align_to_depth(Size(w, h), CV_8UC3, (void*)color_align_to_depth.get_data(), Mat::AUTO_STEP);
            rgb_align.image  = color_image_align_to_depth.clone();
        }

        rs2::depth_frame depth_frame = hole_filter.process(data.get_depth_frame());
        rs2::video_frame color = data.get_color_frame();
        rs2::video_frame infra0 = data.get_infrared_frame(1);
        rs2::video_frame infra1 = data.get_infrared_frame(2);

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat depth_image(Size(w, h), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);//"CV_32FC1"
        Mat color_image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

        Mat ir0(Size(w, h), CV_8UC1, (void*)infra0.get_data(), Mat::AUTO_STEP);
        Mat ir1(Size(w, h), CV_8UC1, (void*)infra1.get_data(), Mat::AUTO_STEP);
        
        std::string im_id = std::to_string(counter);
        counter++;

        depth_image.convertTo(depth_image,CV_32F,depth_factor);

        //assign value to cv image
        std_msgs::Header header;
        header.frame_id  = "left_cam";
        header.stamp  = ros::Time::now();

        depth.header     = header;
        rgb.header       = header;
        rgb_align.header = header;
        image0.header    = header;
        image1.header    = header;
        ci0.header        = header;
        ci1.header       = header;
        ci1.header.frame_id = "right_cam";
        
        depth.image      = depth_image.clone();
        rgb.image        = color_image;
        
        image0.image     = ir0.clone();
        image1.image     = ir1.clone();
   
        pub_depth.publish(depth);
        pub_ir0.publish(image0);
        pub_ir1.publish(image1);
        pub_cam_info0.publish(ci0);
        pub_cam_info1.publish(ci1);
        pub_rgb.publish(rgb);
        if(pub_aligned_rgb)
            pub_rgb_align.publish(rgb_align);


        std::cout<<"publish frame "<<counter<<std::endl;
    }
}

void set_cam_info_default(sensor_msgs::CameraInfo& ci, const rs2_intrinsics& in, const rs2_extrinsics ex){
    ci.width = 640;
    ci.height = 480;
    ci.distortion_model = "radial-tangential";
    ci.K[0] = in.fx;
    ci.K[1] = 0;
    ci.K[2] = in.ppx;
    ci.K[3] = 0;
    ci.K[4] = in.fy;
    ci.K[5] = in.ppy;
    ci.K[6] = 0;
    ci.K[7] = 0;
    ci.K[8] = 1;
    
    ci.D.push_back(0);
    ci.D.push_back(0);
    ci.D.push_back(0);
    ci.D.push_back(0);

    ci.R[0] = 1;
    ci.R[1] = 0;
    ci.R[2] = 0;
    ci.R[3] = 0;
    ci.R[4] = 1;
    ci.R[5] = 0;
    ci.R[6] = 0;
    ci.R[7] = 0;
    ci.R[8] = 1;

    ci.P[0] = in.fx;
    ci.P[1] = 0;
    ci.P[2] = in.ppx;
    ci.P[3] = 0;
    ci.P[4] = 0;
    ci.P[5] = in.fy;
    ci.P[6] = in.ppy;
    ci.P[7] = 0;
    ci.P[8] = 0;
    ci.P[9] = 0;
    ci.P[10] = 1;
    ci.P[11] = 0;
}

void set_cam_info(sensor_msgs::CameraInfo& ci, ros::NodeHandle& nh, int cam_id){
    std::vector<double> mK,mD,mR,mP;
    if(cam_id == 0){
        if(nh.getParam("mD0", mD)){
            ROS_INFO("we get the mD");
        } 
        else{
            ROS_WARN("no mD, warning");
        }
        if(nh.getParam("mR0", mR)){
            ROS_INFO("we get the mR ");
        } 
        else{
            ROS_WARN("no mR, warning");
        }
        if(nh.getParam("mP0", mP)){
            ROS_INFO("we get the mP");
        } 
        else{
            ROS_WARN("no mP, warning");
        }
        if(nh.getParam("mK0", mK)){
            ROS_INFO("we get the mK");
        } 
        else{
            ROS_WARN("no mK, warning");
        }
        ci.header.frame_id = "cam_left";        
    }
    else{
        if(nh.getParam("mD1", mD)){
            ROS_INFO("we get the mD");
        } 
        else{
            ROS_WARN("no mD, warning");
        }
        if(nh.getParam("mR1", mR)){
            ROS_INFO("we get the mR ");
        } 
        else{
            ROS_WARN("no mR, warning");
        }
        if(nh.getParam("mP1", mP)){
            ROS_INFO("we get the mP");
        } 
        else{
            ROS_WARN("no mP, warning");
        }
        if(nh.getParam("mK1", mK)){
            ROS_INFO("we get the mK");
        } 
        else{
            ROS_WARN("no mK, warning");
        }
        ci.header.frame_id = "cam_right";          
    }

    ci.D = mD;

    for(int i = 0 ; i<mR.size(); i++){
        ci.R[i]= mR[i];
    }
    for(int i = 0 ; i<mP.size(); i++){
        ci.P[i]= mP[i];
    }
    for(int i = 0 ; i<mK.size(); i++){
        ci.K[i]= mK[i];
    }
    
    ci.width = 640;
    ci.height = 480;
    ci.distortion_model = "plumb_bob";
}
