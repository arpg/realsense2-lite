Publish D435/D455 realsense raw image without realsense-ros package. Test environment Ubuntu18.04/20.04 LTS, ROS melodic, neotic
### Dependency
Opencv (Should have if you have installed ROS)<br>
[libRealsense2](https://github.com/IntelRealSense/librealsense) <br>
Eigen (Should have if you have installed ROS)<br>
### Install
```
cd your_ros_workspace/src
git clone https://github.com/arpg/realsense2-lite.git
cd ..
catkin_make
```
### Use
Connect your realsense D435, open a terminal
```
roscore
```
In another terminal
```
cd your_ros_workspace
source devel/setup.bash
roslaunch rs2_lite pub_realsense_image.launch
```
You should be able to get 4 topics from ir sensor, depth, rgb, rgb_raw. The `rgb` topic is aligned with the `/image0`. You need tp let the `publish_aligned_rgb` to be `True` to have this topic. `rgb_raw` is the original `rgb` image. Align the rgb with depth causes a delay.
```
/image0
/image1
/depth
/rgb
/rgb_raw
```
The image width is 640X480 and frquency is 30 Hz, you can modify them in the code. But the realsense can only work in certain frame frequency and image size, the default setting is recommended.

### Use the received message
The image topic you receive is encoded in ros message type `cv_bridge::CvImage`. <br>
For ir sensor the [image encoding](http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html) is `mono8`, depth is `32FC1` and rgb is `rgb8`.
If you want to extract the opencv Mat after you receive image. Your subcriber's call back function for ir sensor should be something like
```
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;

    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}
```
for depth image it should be something like
```
void depth_input(const sensor_msgs::ImageConstPtr &depth_input)
{    
    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(depth_input, depth_input->encoding);
    constexpr double kDepthScalingFactor = 0.001;//0.001 for realsense, which means 1000 depth means 1 meters
    if(depth_input->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
        (image_ptr->image).convertTo(image_ptr->image, CV_32FC1, kDepthScalingFactor);
    cv::Mat img = image_ptr->image;
    ros::Time stamp = image_ptr->header.stamp;
}
```
After your callback function you should be able to get an opencv image `img`
