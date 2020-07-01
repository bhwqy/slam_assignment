#ifndef ASSIGNMENT1_UTILS_H
#define ASSIGNMENT1_UTILS_H

#include "CommonHead.h"
#include "LinearLeastSquaresMethod.h"
#include "CoordinateTransform.h"
#include "CalculateDisp.h"
/*calculate the number of rows in a document
 * Para:
 * the pose.txt file path
 * OUTPUT:
 * the number of rows in the txt
 */
int CountLines(string filename);
/*load camera intrinsic parameters and extrinsic parameters for txt;
 *Para:
 * K:   camera intrinsic parameters
 * Pose:camera extrinsic parameters
 */
void TxtDataIO(cv::Mat &K,cv::Mat &Pose);
/*calculate the disparity map for each pair of image in multi-thread
 *Para:
 * disp_img:   the disparity map result
 * left_image: the image in image_2 folder
 * right_inage:the image in image_3 folder
 * row:        rows of image
 * col:        cols of image
 */
void GetDisparityMultiThread(cv::Mat& disp_img, const cv::Mat left_img,const cv::Mat right_img,int row,int col);
/*the API deal with stereo image
 *Para:
 * Index:     the index of the image
 * DispOpenCV:the disparity image result
 */
void GetDisparity(int Index,cv::Mat &DispOpenCV);

/*this function calculate the 3D x,y,z and intensitity
 *INPUT:
 * Point3D:     the x,y,z result
 * DispImg:     the disparity map
 * K:           camera intrinsic parameters
 * Pose:        camera extrinsic parameters
 * AllIntensity:the intensitity of 3D points
 */
void Recover3D(vector<cv::Point3f> &Point3D,cv::Mat &DispImg,cv::Mat &K,cv::Mat &Pose,int index,vector<vector<float>> &AllIntensity);

/*the API to visualize the 3D points
 */
void Viusalization(vector<cv::Point3f> AllPoints,vector<vector<float>> AllIntensity);

/*the test function to check your complementation
 */
void TEST_LINEARLEASTSQUARES();
void TEST_COORDINATETRANSFORM();
#endif //ASSIGNMENT1_UTILS_H
