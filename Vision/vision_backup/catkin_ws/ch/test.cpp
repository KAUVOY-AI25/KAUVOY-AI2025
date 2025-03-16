// ================================================================================
// Main Author: 이창현, 윤수빈
// Recently Modified Date: 2024-12-22 (V1.0)
// Dependency: multicam
// Description: 웹캠 2대를 스티칭 기법을 사용하여 파노라마 뷰로 제작. (30초간 호모그래피 최적화 후 고정) - cpp 버전
// ================================================================================

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <ctime>

class PanoramaNode {
public:
    PanoramaNode()
        : left_cap(0), right_cap(2), best_H(cv::Mat()), calibration_time(30) {
        start_time = std::time(nullptr);
    }

    void processImages() {
        cv::Mat left_image, right_image;
        if (!left_cap.read(left_image) || !right_cap.read(right_image)) {
            std::cerr << "Failed to capture one or both images." << std::endl;
            return;
        }

        // Show input images for debugging
        cv::imshow("Left Image", left_image);
        cv::imshow("Right Image", right_image);
        cv::waitKey(1);

        // During calibration, find the best homography matrix
        if (std::difftime(std::time(nullptr), start_time) < calibration_time) {
            best_H = calibrateHomography(left_image, right_image);
        } else {
            // Use the fixed homography matrix for stitching
            if (!best_H.empty()) {
                cv::Mat panorama = makePanoramaWithFixedH(left_image, right_image);
                if (!panorama.empty()) {
                    cv::imshow("Panorama", panorama);
                    cv::waitKey(1);
                }
            }
        }
    }

private:
    cv::VideoCapture left_cap;
    cv::VideoCapture right_cap;
    cv::Mat best_H;
    int calibration_time;
    std::time_t start_time;

    cv::Mat calibrateHomography(const cv::Mat& left_image, const cv::Mat& right_image) {
        try {
            cv::Mat gray_left, gray_right;
            cv::cvtColor(left_image, gray_left, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_image, gray_right, cv::COLOR_BGR2GRAY);

            // Detect AKAZE features and compute descriptors
            auto akaze = cv::AKAZE::create();
            std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
            cv::Mat descriptors_left, descriptors_right;
            akaze->detectAndCompute(gray_left, cv::noArray(), keypoints_left, descriptors_left);
            akaze->detectAndCompute(gray_right, cv::noArray(), keypoints_right, descriptors_right);

            // Match features using KNN Matcher
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            std::vector<std::vector<cv::DMatch>> knn_matches;
            matcher.knnMatch(descriptors_left, descriptors_right, knn_matches, 2);

            // Apply Lowe's ratio test
            std::vector<cv::DMatch> good_matches;
            for (const auto& match : knn_matches) {
                if (match[0].distance < 0.75 * match[1].distance) {
                    good_matches.push_back(match[0]);
                }
            }

            // Debugging: Check number of matches
            std::cout << "Number of Good Matches (Calibration): " << good_matches.size() << std::endl;
            if (good_matches.size() < 10) {
                std::cerr << "Not enough good matches for homography calibration." << std::endl;
                return best_H;
            }

            std::vector<cv::Point2f> points_left, points_right;
            for (const auto& match : good_matches) {
                points_left.push_back(keypoints_left[match.queryIdx].pt);
                points_right.push_back(keypoints_right[match.trainIdx].pt);
            }

            if (points_left.size() < 4 || points_right.size() < 4) {
                std::cerr << "Not enough points for homography calibration." << std::endl;
                return best_H;
            }

            cv::Mat H = cv::findHomography(points_right, points_left, cv::RANSAC, 5.0);
            std::cout << "Calibrated Homography Matrix: \n" << H << std::endl;
            return H;
        } catch (const std::exception& e) {
            std::cerr << "Error during homography calibration: " << e.what() << std::endl;
            return best_H;
        }
    }

    cv::Mat makePanoramaWithFixedH(const cv::Mat& left_image, const cv::Mat& right_image) {
        try {
            if (best_H.empty()) {
                std::cerr << "Homography matrix is not calibrated." << std::endl;
                return cv::Mat();
            }

            int height = std::max(left_image.rows, right_image.rows);
            int width = left_image.cols + right_image.cols;
            cv::Mat result;
            cv::warpPerspective(right_image, result, best_H, cv::Size(width, height));
            left_image.copyTo(result(cv::Rect(0, 0, left_image.cols, left_image.rows)));
            return result;
        } catch (const std::exception& e) {
            std::cerr << "Error during panorama stitching with fixed H: " << e.what() << std::endl;
            return cv::Mat();
        }
    }
};

int main() {
    PanoramaNode panorama_node;
    while (true) {
        panorama_node.processImages();
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
