
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(first_time==0){
    std::cout<<"Starting setup\n";
    setup_inference();
    std::cout<<"Finished setup\n";
    first_time = 1;
  }
  try
  {
    cv::Mat image_show;
    image_show = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("view", image_show);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

//~/manoj_work/ValveDetectCatkin/catkin_ws

int main(int argc, char** argv) {

    ros::init(argc, argv, "ObiWan_listener");
    ros::NodeHandle nh;
    cv::namedWindow("Image viewer");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("Image viewer");
    