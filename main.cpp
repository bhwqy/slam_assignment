#include "CommonHead.h"
#include "utils.h"
#include "CoordinateTransform.h"
int main() {

    TEST_LINEARLEASTSQUARES();
    TEST_COORDINATETRANSFORM();

    //go!!
    cv::Mat K,Pose;
    TxtDataIO(K,Pose);
    int NumberImage=Pose.rows;
    //All 3D point and intenstity
    vector<cv::Point3f> AllPoints;
    vector<vector<float>> AllIntensity;

    //the main loop deals with all pictures
    for(int i=0;i<NumberImage;++i){
        //get disparity map
        cv::Mat DispImg;
        GetDisparity(i,DispImg);
        //calculate 3D point and intenstity
        vector<cv::Point3f> Point3D;
        vector<vector<float>> Intensity;
        Recover3D(Point3D,DispImg,K,Pose,i,AllIntensity);
        vector<cv::Point3f> wPoint3D=CameraToWorld(i,Point3D,Pose);
        AllPoints.insert(AllPoints.end(),wPoint3D.begin(),wPoint3D.end());
    }
#ifdef DEBUG_INFORMATION
    cout<<"[DEBUG INFORMATION] the number of points:"<<AllPoints.size()<<endl;
    cout<<"[DEBUG INFORMATION] waiting for writing data into points.ply file.Do not stop the programme!"<<AllPoints.size()<<endl;
#endif
    Viusalization(AllPoints,AllIntensity);
    return 0;
}