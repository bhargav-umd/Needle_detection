#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static const std::string OPENCV_WINDOW = "PI1";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

 public:
    ImageConverter() : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ =
            it_.subscribe("/camera/image", 1, &ImageConverter::imageCb, this);
        if (image_sub_ == NULL) {
            ROS_ERROR("Images not getting read properly");
        }
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() { cv::destroyWindow(OPENCV_WINDOW); }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr =
                cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Draw an example circle on the video stream
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //  cv::circle(cv_ptr->image, cv::Point(300, 300), 100,
        //           CV_RGB(255, 0, 0));
        // cv::rotate(cv_ptr->image, cv_ptr->image, cv::ROTATE_90_CLOCKWISE);
        // Update GUI Window
        // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        // cv::waitKey(1);

        cv::Mat img = cv_ptr->image;
        cv::Mat needle_detect_ = img.clone();

        cv::Mat edge, needle, edge_image;
        cv::Canny(img, edge, 50, 100, 3);
        edge.convertTo(edge_image, CV_8U);

        std::vector<cv::Vec4i> lines;
        HoughLinesP(edge_image, lines, 1, CV_PI / 180, 30, 30, 10);
        std::cout << "the number of lines are " << lines.size() << std::endl;
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            line(needle_detect_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                 cvScalar(0, 0, 255), 1, CV_AA);
        }
        cv::imshow("edge", edge);
        cv::imshow("needle", needle_detect_);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
