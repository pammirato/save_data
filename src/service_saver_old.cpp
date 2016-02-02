#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>



//#include <mutex>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <math.h>


#include <iostream>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>



//#include <ros/spinner.h>
#include <ros/callback_queue.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>





//#include <kinect2_definitions.h>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}



  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image,sensor_msgs::Image> ExactSyncPolicy;

//pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cur_cloud (new pcl::PointCloud<pcl::PointXYZ>);

cv::Mat camera_matrix_rgb;
cv::Mat lookupX, lookupY;


const std::string base_save_path_1 = "/home/ammirato/Documents/Kinect/Data/";
const std::string base_save_path_2 = "/Keyboard/Test/";
std::string rgb_save_path = "rgb/";
std::string depth_save_path = "depth/";
std::string raw_depth_save_path = + "raw_depth/";
const std::string rgb_save_name =  "rgb/rgb";
const std::string depth_save_name =  "depth/depth";
const std::string raw_depth_save_name= "raw_depth/raw_depth";

const std::string image_extension = ".png";

std::string base_name = "kinect2";

int counter = -1;
bool save_images = false;
bool save_rgb = false;
bool save_depth = false;
bool save_raw_depth = false;

//std::mutex lock;
pthread_mutex_t mutex;

cv::Mat rgb;
cv::Mat depth;
cv::Mat raw_depth;

std::vector<int> compression_params;
    

image_transport::SubscriberFilter *rgb_filter_sub, *depth_filter_sub, *raw_depth_filter_sub; 
message_filters::Synchronizer<ExactSyncPolicy> *syncExact;

ros::Subscriber camera_info_sub;
ros::Subscriber odom_sub;

std::string timestamp_sec;
std::string timestamp_nsec;


bool  saved;

double curx = 0;
double cury = 0;
double curz = 0;
double curo = 0;





void odom_callback(const nav_msgs::Odometry::ConstPtr odom_msg)
{
  double x = odom_msg->pose.pose.position.x;
  double y = odom_msg->pose.pose.position.y;
  double z = odom_msg->pose.pose.position.z;

  double qx = odom_msg->pose.pose.orientation.x;
  double qy = odom_msg->pose.pose.orientation.y;
  double qz = odom_msg->pose.pose.orientation.z;
  double qw = odom_msg->pose.pose.orientation.w;

  curx = x;
  cury = y;
  curo = acos(qw) * 180 / M_PI;


  if(qz < 0)
    curo *= -1;
//  ROS_INFO("Poisition(x,y,z): %f, %f, %f\n ", x,y,z);

//  ROS_INFO("Quaternion: %f, %f, %f, %f\n",qx,qy,qz,qw);
//  ROS_INFO("Approx Angle: %f\n\n\n",2*acos(qw));

}//odom callback


void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr camera_info_rgb)
{
  /*ROS_INFO("INFOR CALLBACK");
  double *itC = camera_matrix_rgb.ptr<double>(0, 0); 
  for(size_t i = 0; i < 9; ++i, ++itC)
  {   
    *itC = camera_info_rgb->K[i];
    std::cout << *itC << std::endl;
  }*/ 
}//camera info callback












void images_callback(const sensor_msgs::Image::ConstPtr rgb_msg, const sensor_msgs::Image::ConstPtr depth_msg, const sensor_msgs::Image::ConstPtr raw_depth_msg)
{
  //ROS_INFO("images -callback");
  //lock.lock();  
  pthread_mutex_lock(&mutex);
  //ROS_INFO("images -callback got lock");
  if(save_images)
  {
    ROS_INFO("SAVE CALLBACK");
    try
    {
      //convert message too opencv mat
      rgb = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding)->image;
      cv::imshow("rgb",rgb );
      cv::resizeWindow("rgb",432, 768);
      std::string timestamp_sec =patch::to_string(rgb_msg->header.stamp.sec); 
      std::string timestamp_nsec=patch::to_string(rgb_msg->header.stamp.nsec); 
      cv::imwrite(rgb_save_path + timestamp_sec + "_" + timestamp_nsec
              + "_" + patch::to_string(curx)
              + "_" + patch::to_string(cury)
              + "_" + patch::to_string(curz)
              + "_" + patch::to_string(curo)
               + image_extension, rgb, compression_params);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to rgb.", rgb_msg->encoding.c_str());
    }//catch


    //DEPTH image
    try
    {
      //convert message too opencv mat
      depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;
      cv::imshow("depth",depth );
      cv::resizeWindow("depth",432, 768);
      std::string timestamp_sec =patch::to_string(rgb_msg->header.stamp.sec); 
      std::string timestamp_nsec=patch::to_string(rgb_msg->header.stamp.nsec); 
      cv::imwrite(depth_save_path + timestamp_sec + "_" + timestamp_nsec  + image_extension, depth, compression_params);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to depth.", depth_msg->encoding.c_str());
    }//catch



    //RAW DEPTH image
    try
    {
      //convert message too opencv mat
      raw_depth = cv_bridge::toCvShare(raw_depth_msg, raw_depth_msg->encoding)->image;
      std::string timestamp_sec =patch::to_string(rgb_msg->header.stamp.sec); 
      std::string timestamp_nsec=patch::to_string(rgb_msg->header.stamp.nsec); 
      cv::imwrite(raw_depth_save_path + timestamp_sec + "_" + timestamp_nsec  + image_extension, raw_depth, compression_params);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to raw depth'.", raw_depth_msg->encoding.c_str());
    }//catch


    save_images = false;
  }//if save images
  else
  {
    ros::Duration(.01).sleep();
  }//else not saveimages
  //lock.unlock();
  pthread_mutex_unlock(&mutex);



}//callback













bool save(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

  //lock.lock();
  pthread_mutex_lock(&mutex);
  save_images = true;
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
  ROS_INFO("SAVE SERVICE");
  //lock.unlock();
  pthread_mutex_unlock(&mutex);

  return true;
}






void create_lookup(size_t width, size_t height)
{
  const float fx = 1.0f / camera_matrix_rgb.at<double>(0, 0);
  const float fy = 1.0f / camera_matrix_rgb.at<double>(1, 1);
  const float cx = camera_matrix_rgb.at<double>(0, 2);
  const float cy = camera_matrix_rgb.at<double>(1, 2);
  float *it;

  std::cout << fx << ","<<fy<<","<<cx<<","<<cy<<std::endl;


  lookupY = cv::Mat(1, height, CV_32F);
  it = lookupY.ptr<float>();
  for(size_t r = 0; r < height; ++r, ++it)
  {
    *it = (r - cy) * fy;
  }

  lookupX = cv::Mat(1, width, CV_32F);
  it = lookupX.ptr<float>();
  for(size_t c = 0; c < width; ++c, ++it)
  {
    *it = (c - cx) * fx;
  }
}





void update_cur_cloud()
{

  cur_cloud->height = rgb.rows;
  cur_cloud->width = rgb.cols;
  cur_cloud->is_dense = true;
  cur_cloud->points.resize(cur_cloud->height * cur_cloud->width);
  create_lookup(rgb.cols, rgb.rows);


  const float badPoint = std::numeric_limits<float>::quiet_NaN();
  #pragma omp parallel for
  for(int r = 0; r < depth.rows; ++r)
  {   
    pcl::PointXYZ *itP = &cur_cloud->points[r * depth.cols];
    const uint16_t *itD = depth.ptr<uint16_t>(r); 
    const cv::Vec3b *itC = rgb.ptr<cv::Vec3b>(r);
    const float y = lookupY.at<float>(0, r);
    const float *itX = lookupX.ptr<float>(); 

    for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
    { 
      register const float depthValue = *itD / 1000.0f;
      // Check for invalid measurements
      /*if(isnan(depthValue) || depthValue <= 0.001)
      {
        // not valid
        //itP->x = itP->y = itP->z = badPoint;
       // itP->rgba = 0;
        continue;
      }*/
      itP->z = depthValue;
      itP->x = *itX * depthValue;
      itP->y = y * depthValue;
      
//      std::cout << itP->x << "," << itP->y << "," << itP->z << std::endl;
      //itP->b = itC->val[0];
      //itP->g = itC->val[1];
      //itP->r = itC->val[2];
      //itP->a = 0;
    }
  }//for r

}//make_point_cloud 











int main(int argc, char **argv)
{
/*  std::string node_name = "saver";
  if(argc > 1)
    node_name = std::string(argv[1]);
  ROS_INFO("name: %s", node_name.c_str());
*/

  ros::init(argc, argv, "saver",  ros::init_options::AnonymousName);

  ros::NodeHandle nh = ros::NodeHandle("~");
  nh.getParam("base_name", base_name);


  mkdir((base_save_path_1+ base_name + base_save_path_2).c_str(),0777);
  mkdir((base_save_path_1+ base_name + base_save_path_2 + rgb_save_path).c_str(),0777);
  mkdir((base_save_path_1+ base_name + base_save_path_2 + depth_save_path).c_str(),0777);
  mkdir((base_save_path_1+ base_name + base_save_path_2 + raw_depth_save_path).c_str(),0777);

/*
  rgb_save_path = base_save_path_1 +  base_name  + base_save_path_2 + rgb_save_name;
  depth_save_path = base_save_path_1 +  base_name +  base_save_path_2 + depth_save_name;
  raw_depth_save_path = base_save_path_1 +  base_name + base_save_path_2 + raw_depth_save_name;


  ROS_INFO("%s", rgb_save_path.c_str());



  std::string ns = "/" + base_name;

  cv::namedWindow("rgb",CV_WINDOW_NORMAL);
  cv::namedWindow("depth", CV_WINDOW_NORMAL);
  cv::startWindowThread();

  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  image_transport::ImageTransport it(nh);

  image_transport::TransportHints rgb_hints("raw");
  image_transport::TransportHints depth_hints("raw");
  image_transport::TransportHints raw_depth_hints("raw");

  std::string rgb_topic = ns + K2_TOPIC_IMAGE_COLOR + K2_TOPIC_RAW;
  std::string depth_topic = ns + K2_TOPIC_HIRES_DEPTH + K2_TOPIC_RAW;
  std::string raw_depth_topic = ns + K2_TOPIC_IMAGE_DEPTH + K2_TOPIC_RAW;
  std::string camera_info_topic = ns + K2_TOPIC_IMAGE_COLOR + K2_TOPIC_INFO;
  int queue_size = 3;

  rgb_filter_sub = new image_transport::SubscriberFilter(it, rgb_topic,queue_size, rgb_hints);
  depth_filter_sub = new image_transport::SubscriberFilter(it, depth_topic,queue_size, depth_hints);
  raw_depth_filter_sub = new image_transport::SubscriberFilter(it, raw_depth_topic,queue_size, raw_depth_hints);


  syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queue_size), *rgb_filter_sub, *depth_filter_sub,*raw_depth_filter_sub);
  syncExact->registerCallback(boost::bind(&images_callback, _1, _2 ,_3));
*/

//  camera_info_sub = nh.subscribe(camera_info_topic, 1, camera_info_callback);


  /*std::vector<double> rgb_info_matrix;
  rgb_info_matrix.push_back(1070);
  rgb_info_matrix.push_back(0);
  rgb_info_matrix.push_back(927.269);
  rgb_info_matrix.push_back(0);
  rgb_info_matrix.push_back(1069.12);
  rgb_info_matrix.push_back(545.761);
  rgb_info_matrix.push_back(0);
  rgb_info_matrix.push_back(0);
  rgb_info_matrix.push_back(1);
  camera_matrix_rgb = cv::Mat::zeros(3, 3, CV_64F);
  double *itC = camera_matrix_rgb.ptr<double>(0, 0); 
  for(size_t i = 0; i < 9; ++i, ++itC)
  {
    std::cout << "INFOOOOOOOO:   " << rgb_info_matrix.at(i) <<std::endl;   
    *itC = rgb_info_matrix.at(i);
  } 


 
//  do_icp();


  //cloud/icp stuff
 //   cur_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  //  prev_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  odom_sub = nh.subscribe("/rtabmap/odom",1,odom_callback);

  ros::ServiceServer service =  nh.advertiseService(ns + "/save_images", save);

  if (pthread_mutex_init(&mutex, NULL) != 0)
  {
    printf("\n mutex init failed\n");
    return 1;
  }

  //while(ros::ok()){
  //  ros::spinOnce();
  //}
  ros::spin(); 

  cv::destroyWindow("rgb");
  cv::destroyWindow("depth");

*/

  ros::shutdown();
}

