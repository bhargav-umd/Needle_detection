#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

int main() {
    cv::Mat img =
        cv::imread("4_PI1_screenshot_29.11.2018.png", CV_LOAD_IMAGE_COLOR);
    cv::Mat needle_detect_ = img.clone();

    cv::Vec3f circ(354, 275, 100);

    cv::Mat1b mask(img.size(), uchar(0));
    circle(mask, cv::Point(circ[0], circ[1]), circ[2], cvScalar(255),
           CV_FILLED);

    // Compute the bounding box
    cv::Rect bbox(circ[0] - circ[2], circ[1] - circ[2], 2 * circ[2],
                  2 * circ[2]);

    // Create a black image
    cv::Mat3b res(img.size(), cv::Vec3b(0, 0, 0));

    // Copy only the image under the white circle to black image
    img.copyTo(res, mask);

    // Crop according to the roi
    res = res(bbox);

    cv::imshow("cropped circle", res);

    cv::Mat edge, needle, edge_image;
    cv::Canny(res, edge, 50, 140, 3);
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
        std::cout << " length of line number: " << i << " is : " << leng
                  << std::endl;
        line(needle_detect_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
             cvScalar(255, 255, 255), 2, CV_AA);
        std::cout << "the coordinates of the lines are: "
                  << cv::Point(l[0], l[1]) << " and " << cv::Point(l[2], l[3])
                  << std::endl;
        if (leng < 40 && leng > 30) {
            line(needle_detect_, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                 cvScalar(0, 0, 255), 1, CV_AA);
        }
    }
    cv::imshow("edge", edge);
    cv::imshow("needle", needle_detect_);
    cv::waitKey(0);
    return 0;
}
