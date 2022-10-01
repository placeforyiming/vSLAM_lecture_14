#include <opencv2/opencv.hpp>
#include <string>
using std::cout;
using std::endl;

int main(int argc, char **argv) {

    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image;
    image = cv::imread(argv[1]);
    if (image.data == nullptr) {
        cout << "img file does not exist" << endl;
        return  0;
    }
    int rows = image.rows, cols = image.cols;

    // the undistorted result
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);
    // go through every pixel to 
    for (int v = 0; v < rows; v++) {
        for (int u = 0; u < cols; u++) {
            // calculated undistorted position
            double x = (u - cx) / fx, y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;
            // nearest interpolation
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }
    }

    cv::imshow("distorted", image);
    cv::imshow("undistorted", image_undistort);
    cv::waitKey();
    return 0;
}
