#ifndef ASSIGNMENT1_CALCULATEDISP_H
#define ASSIGNMENT1_CALCULATEDISP_H

#include "CommonHead.h"
/*calculate the disparity for each pixel in the left image.Read the PDF tutorial for more information
 *INPUT:
 * left_img: image catched by left camera
 * right_img:image catched by right camera
 * row:      the pixel's location(row) in the left image
 * col:      the pixel's location(col) in the left image
 *OUTPUT:
 * D:        the disparity of this pixel
 */
int CalculateDisp(const cv::Mat left_img,const cv::Mat right_img,int row,int col);
#endif //ASSIGNMENT1_CALCULATEDISP_H
