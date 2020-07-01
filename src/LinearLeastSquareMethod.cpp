#include "LinearLeastSquaresMethod.h"
/*the Linear least squares solver:A^T*A*x=A^-1*b
 *INPUT:
 * DispImg:the disparity map
 * K:      camera intrinsic parameters
 * row:    the row of the pixel in left image
 * col:    the col of the pixel in left image
 *OUTPUT:
 * lamda:  the 2X1 matrix contains lamda0 and lamda1
 * =================================================
 * TIPS:row stands for the pixel coordinate u,col stands for the pixel coordinate v
        the pixel coordinate is expressed as [u,v]
 */
cv::Mat LinearLeastSquares(cv::Mat &DispImg, cv::Mat &K, int row, int col) {
    cv::Mat lamda;
    const float baseline = 0.54;
    cv::Mat A(3, 2, CV_32FC1);
    A.at<float>(0, 0) = row;
    A.at<float>(1, 0) = col;
    A.at<float>(2, 0) = 1;
    A.at<float>(0, 1) = -row + DispImg.at<float>(row, col);
    A.at<float>(1, 1) = -col;
    A.at<float>(2, 1) = -1;
    cv::Mat B = cv::Mat::zeros(3, 1, CV_32FC1);
    B.at<float>(0, 0) = baseline;
    lamda = (A.t() * A).inv() * A.t() * K * B;
    return lamda;
}
