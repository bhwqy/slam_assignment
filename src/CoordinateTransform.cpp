#include "CoordinateTransform.h"
/*this function transform a 3D point in camera cordinate to world cordinate so that
 *we can visualize all 3D points in a same coordinate system.
 *INPUT:
 * index:   the index of the current image pair
 * cPoint3D:3D points in camera frame.
 * Pose:    the NX12 matrix of the camera pose.each line stands for a corresponding pose(R|T)
 *OUTPUT:
 * wPoint3D:3D points in world frame.
 *==================================
 * TIPS:wP=Rwc*cP+Twc.we has given you the 4X4 matrix Twc, so you need only focus on how to transform the cordinate.
 */
vector<cv::Point3f> CameraToWorld(int index, vector<cv::Point3f> &cPoint3D, cv::Mat &Pose){
    cv::Mat Twc = Pose.row(index);
    Twc = Twc.reshape(0,3);
    vector<cv::Point3f> wPoint3D;
    cv::Mat R, t;
    Twc(cv::Range(0, 3), cv::Range(0, 3)).copyTo(R);
    Twc(cv::Range(0, 3), cv::Range(3, 4)).copyTo(t);
    for (int i = 0; i < cPoint3D.size(); ++i) {
        cv::Mat point(3, 1, CV_32FC1);
        point.at<float>(0) = cPoint3D[i].x;
        point.at<float>(1) = cPoint3D[i].y;
        point.at<float>(2) = cPoint3D[i].z;
        cv::Mat ans = R * point + t;
        wPoint3D.push_back(cv::Point3f(
            ans.at<float>(0),
            ans.at<float>(1),
            ans.at<float>(2)
        ));
    }
    return wPoint3D;
}
