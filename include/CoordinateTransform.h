#ifndef ASSIGNMENT1_COORDINATETRANSFORM_H
#define ASSIGNMENT1_COORDINATETRANSFORM_H

#include <CommonHead.h>
/*this function transform a 3D point in camera cordinate to world cordinate so that
 *we can visualize all 3D points in a same coordinate system.
 *INPUT:
 * index:   the index of the current image pair
 * cPoint3D:3D points in camera frame.
 * Pose:    the NX12 matrix of the camera pose.each line stands for a corresponding pose(R|T)
 *OUTPUT:
 * wPoint3D:3D points in world frame.
 *==================================
 * TIPS:wP=Rwc*cP+Twc.we has given you the 4X4 matrix Twc, so you need only focus on how to transform the 3D cordinate.
 */
vector<cv::Point3f> CameraToWorld(int index,vector<cv::Point3f> &Point3D,cv::Mat &Pose);

#endif //ASSIGNMENT1_COORDINATETRANSFORM_H
