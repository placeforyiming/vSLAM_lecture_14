#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;

int main(int argc, char **argv) {
    // read an image, print basic information
    cv::Mat image;
    image = cv::imread(argv[1]);
    if (image.data == nullptr) {
        cout << "img file does not exist" << endl;
        return  0;
    }
    cout << "img width is: " << image.cols << ", height is: "
         << image.rows << ", n_channel is: " << image.channels() <<endl;
    cv::imshow("image", image);
    cv::waitKey(0);

    // go through the image, and counting the time cost
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++) {
        // using cv::Mat::ptr get the pointer of each image row
        // the y-th row ptr
        unsigned char *row_ptr = image.ptr<unsigned char>(y);
        for (size_t x = 0; x < image.cols; x++) {
            // ptr to the channels of each position
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
            for (int c = 0; c != image.channels(); c++) {
                unsigned char data = data_ptr[c];
            }
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    cout << "time cost of walking through the entire image is: " << time_used.count() << "s" << endl;

    // image copy default, will not deep copy image data, but only reference
    cv::Mat image_another = image;
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
    cv::imshow("image", image);
    cv::waitKey(0);

    // true copy
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}
