#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Detected Circles";

class ImageListener
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  cv::Mat hsv_image_, red_hue_image_, lower_red_hue_range_, upper_red_hue_range_;
  cv::Mat out_image_;
  
public:
  ImageListener() : it_(nh_)
  {
    // Listen for image messages on a topic and setup callback
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageListener::imageCb, this);

    // Open HighGUI Window
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageListener()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
};

void ImageListener::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert ROS input image message to CvImagePtr
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Copy input image
  out_image_ = cv::Mat(cv_ptr->image).clone();

  // Convert input image from BGR to HSV
  cv::cvtColor(cv_ptr->image, hsv_image_, CV_BGR2HSV);

  // Threshold the HSV image, keep only the red pixels
  cv::Mat lower_red_hue_range_;
  cv::Mat upper_red_hue_range_;
  cv::inRange(hsv_image_, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range_);
  cv::inRange(hsv_image_, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range_);

  // Combine the above two images
  cv::addWeighted(lower_red_hue_range_, 1.0, upper_red_hue_range_, 1.0, 0.0, red_hue_image_);

  // Smooth image to avoid falsely detected circles
  cv::GaussianBlur(red_hue_image_, red_hue_image_, cv::Size(9, 9), 2, 2);

  // The vector circles will hold the position and radius of the detected circles
  std::vector<cv::Vec3f> circles;

  // Detect circles that have a radius between 50 and 400 that are a minimum of red_hue_image.rows/2 pixels apart
  cv::HoughCircles(red_hue_image_, circles, CV_HOUGH_GRADIENT, 1, red_hue_image_.rows/2, 80, 40, 50, 400);

  for(size_t i = 0; i < circles.size(); i++)
  {
    // Gets the position of center and radius of circle detected
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);

    // Draw the circle center
    cv::circle(out_image_, center, 3,  cv::Scalar(0,255,0), -1, 8, 0);

    // Draw the circle outline
    cv::circle(out_image_, center, radius+1, cv::Scalar(0,0,255), 2, 8, 0);

    // Debugging Output
    ROS_INFO("x: %d y: %d r: %d", center.x, center.y, radius);
  }

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, out_image_);
  cv::waitKey(3);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ImageListener il;
  ros::spin();
  return 0;
}
