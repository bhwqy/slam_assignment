#ifndef ASSIGNMENT1_LINEARLEASTSQUARESMETHOD_H
#define ASSIGNMENT1_LINEARLEASTSQUARESMETHOD_H

#include "CommonHead.h"
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
cv::Mat LinearLeastSquares(cv::Mat &DispImg,cv::Mat &K,int row,int col);

#endif //ASSIGNMENT1_LINEARLEASTSQUARESMETHOD_H
