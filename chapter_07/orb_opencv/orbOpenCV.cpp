#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <vector>

using std::cout;
using std::endl;
using std::vector;
using std::minmax_element;
using std::max;

using cv::Mat;
using cv::imread;
using cv::Ptr;
using cv::KeyPoint;
using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::DescriptorMatcher;
using cv::imshow;
using cv::drawKeypoints;
using cv::Scalar;
using cv::DrawMatchesFlags;
using cv::ORB;
using cv::DMatch;
using cv::waitKey;

int main(int argc, char **argv) {
    if (argc != 3) {
        cout << "usage : feature_extraction img1 img2" << endl;
        return 1;
    }

    // read image 
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    // initialization
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matchers = DescriptorMatcher::create("BruteForce-Hamming");

    // detect oriented fast corner point
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // calculate descriptor
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds." << endl;
    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimg1);

    // using hamming distance to matching descriptors
    vector<DMatch> matches;
    t1 = std::chrono::steady_clock::now();
    matchers->match(descriptors_1, descriptors_2, matches);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " second. " << endl;

    // filter mached points, if the distance is too far, remove the matches
    auto min_max = minmax_element(matches.begin(), matches.end(),
        [](const DMatch &m1, const DMatch &m2) {return m1.distance < m2.distance;});
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);

    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (matches[i].distance <= max(2*min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }
    }

    // draw the result
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1,  img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1,  img_2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);
    return 0;
}