#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

int main() {
    cv::Mat img =
        cv::imread("4_PI1_screenshot_29.11.2018.png", CV_LOAD_IMAGE_COLOR);
    cv::Mat needle_detect_ = img.clone();

    cv::Mat roi;   // Matrix to save image
                   // creating empty mask image
    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
    cv::circle(mask, cv::Point(354, 275), 100, cvScalar(255, 255, 255), -1, 8,
               0);           //-1 means filled
    img.copyTo(roi, mask);   // copy values of img to dst if mask is > 0
    cv::imshow("roi", roi);

    cv::Mat edge, needle, edge_image;
    cv::Canny(roi, edge, 50, 60, 3);
    edge.convertTo(edge_image, CV_8U);
    // Draw an example circle on the video stream
    cv::circle(needle_detect_, cv::Point(354, 275), 100, CV_RGB(255, 0, 0));
    // cv::rotate(cv_ptr->image, cv_ptr->image, cv::ROTATE_90_CLOCKWISE);
    // Update GUI Window cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(1);

    std::vector<cv::Vec4i> lines;
    HoughLinesP(edge_image, lines, 1, CV_PI / 180, 30, 30, 10);
    std::cout << "the number of lines are" << lines.size() << std::endl;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        double leng = cv::norm(cv::Point(l[0], l[1]) - cv::Point(l[2], l[3]));

        if (leng < 40 && leng > 30) {
            line(needle_detect_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                 cvScalar(0, 0, 255), 1, CV_AA);
            std::cout << " length of line number: " << i << " is : " << leng
                      << std::endl;
            // line(needle_detect_, cv::Point(l[0], l[1]), cv::Point(l[2],
            // l[3]),
            //   cvScalar(255, 255, 255), 2, CV_AA);

            std::cout << "the coordinates of the lines are: "
                      << cv::Point(l[0], l[1]) << " and "
                      << cv::Point(l[2], l[3]) << std::endl;
        }
    }
    cv::imshow("edge", edge);
    cv::imshow("needle", needle_detect_);
    cv::waitKey(0);
    return 0;
}
