#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

#include "std_msgs/Float64MultiArray.h"

static const std::string OPENCV_WINDOW = "PI1";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher mid_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("needle_locate", 1);

 public:
    ImageConverter() : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ =
            it_.subscribe("/camera/image", 1, &ImageConverter::imageCb, this);
        if (image_sub_ == NULL) {
            ROS_ERROR("Images not getting read properly");
        }
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        // image_pub_ = it_.advertise("midfinder/point", 1);
        cv::namedWindow(OPENCV_WINDOW);
    }
    // mid_pub_ = nh_.advertise("needle_locate", 1);

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
        cv::Point midi;
        midi.x = 0;
        midi.y = 0;

        cv::Mat img = cv_ptr->image;
        cv::Mat roi;   // Matrix to save image
                       // creating empty mask image
        cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
        cv::circle(mask, cv::Point(354, 275), 80, cvScalar(255, 255, 255), -1,
                   8,
                   0);           //-1 means filled
        img.copyTo(roi, mask);   // copy values of img to dst if mask is > 0
        cv::imshow("roi", roi);

        cv::Mat needle_detect_ = img.clone();
        cv::Mat edge, needle, edge_image;
        cv::Canny(roi, edge, 50, 140, 3);
        edge.convertTo(edge_image, CV_8U);
        cv::imshow("edge", edge_image);

        std::vector<cv::Vec4i> lines;
        HoughLinesP(edge_image, lines, 1, CV_PI / 180, 10, 10, 10);
        std::cout << "the number of lines are " << lines.size() << std::endl;
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            //            line(needle_detect_, cv::Point(l[0], l[1]),
            //            cv::Point(l[2], l[3]),
            //                 cvScalar(0, 0, 255), 1, CV_AA
            double leng =
                cv::norm(cv::Point(l[0], l[1]) - cv::Point(l[2], l[3]));
            //            std::cout << " length of line number: " << i << " is
            //            :" << leng
            //                      << std::endl;
            //
            if (leng < 44.5 && leng > 39) {
                line(needle_detect_, cv::Point(l[0], l[1]),
                     cv::Point(l[2], l[3]), cvScalar(255, 255, 255), 2, CV_AA);
                midi = (cv::Point(l[0], l[1]) + cv::Point(l[2], l[3])) * .5;
                circle(needle_detect_, midi, 4, cvScalar(255, 0, 0), -1, 8, 0);
                std::cout << "Mid point is " << midi << std::endl;
                std::cout << "---------------------" << std::endl;
                std::cout << "the coordinates of the lines are: "
                          << cv::Point(l[0], l[1]) << " and "
                          << cv::Point(l[2], l[3]) << std::endl;
                std::cout << "---------------------" << std::endl;
            };
        }
        cv::imshow("edge", edge);
        cv::imshow("needle", needle_detect_);

        std::cout << "the mid point is xxxxxx:  " << midi.x << "and y is "
                  << midi.y << std::endl;

        std_msgs::Float64MultiArray x;

        x.data.push_back(midi.x);
        x.data.push_back(midi.y);

        image_pub_.publish(cv_ptr->toImageMsg());
        mid_pub_.publish(x);

        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;

    ros::spin();
    return 0;
}
