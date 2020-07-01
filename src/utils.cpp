#include "utils.h"
int PATCHRADIUS=5;
int MINDISP=5;
int MAXDISP=50;
int patchsize=2*PATCHRADIUS+1;

int CountLines(std::string filename){
    ifstream ReadFile;
    int n=0;
    char line[512];
    ReadFile.open(filename,ios::in);
    if(ReadFile.fail())
    {
        cout<<"[ERROR]can not find the txt of pose,please check the file path!"<<endl;
        return 0;
    }
    else//文件存在
    {
        while(!ReadFile.eof())
        {
            ReadFile.getline(line,512,'\n');
            n++;
        }
    }
    ReadFile.close();
    return n-1;
}
void TxtDataIO(cv::Mat &K,cv::Mat &Pose){
    //load K
    Eigen::Matrix3d k;
    string file="../DataSet/K.txt";
    ifstream fin;
    fin.open(file);
    if(!fin){
        cout<<"[ERROR]can not find the txt of K,please check the file path!"<<endl;
        return;
    }
    for(int i=0;i<3;++i){
        double x1,x2,x3;
        fin>>x1>>x2>>x3;
        k.row(i)<<x1,x2,x3;
    }
    fin.close();
    fin.clear();
    cv::eigen2cv(k,K);
    K.convertTo(K,CV_32FC1);
    //zoom the image to 1/2 in order to speed up
    //so we adjust the camera intrinsic parameters here
    K/=2;
    K.at<float>(2,2)=1;
#ifdef DEBUG_INFORMATION
    cout<<"[DEBUG INFORMATION]"<<endl<<"K:"<<endl<<K<<endl;
#endif
    //load pose
    file="../DataSet/00.txt";
    int LineNum=CountLines(file);
    Eigen::MatrixXd pose(LineNum,12);
    fin.open(file);
    for(int i=0;i<LineNum;++i){
        double  x1,x2,x3,x4,x5,x6,x7,x8,
                x9,x10,x11,x12;
        fin>>x1>>x2>>x3>>x4>>x5>>x6>>x7>>x8>>
           x9>>x10>>x11>>x12;
        pose.row(i)<<x1,x2,x3,x4,x5,x6,x7,x8,
                x9,x10,x11,x12;
    }
    fin.close();
    fin.clear();
    eigen2cv(pose,Pose);
    Pose.convertTo(Pose,CV_32FC1);
#ifdef DEBUG_INFORMATION
    cout<<"[DEBUG INFORMATION] "<<"Pose rows:"<<Pose.rows<<" "<<"Pose cols:"<<Pose.cols<<endl;
#endif
}
void GetDisparityMultiThread(cv::Mat& disp_img, const cv::Mat left_img,const cv::Mat right_img,int row,int cols){
    for(int col=(MAXDISP+PATCHRADIUS);col<=(cols-PATCHRADIUS-1);++col) {
        int D=CalculateDisp(left_img,right_img,row,col);
        if(D==-1)
            continue;
        disp_img.at<float>(row, col) = D;
    }
}
void GetDisparity(int Index,cv::Mat &DispOpenCV){
    char num[6];
    sprintf(num,"%06d",Index);
    string num_=num;

    string filepath = "../DataSet/image_2/" + num_ + ".png";
    cv::Mat img1_ = cv::imread(filepath);

    filepath = "../DataSet/image_3/" + num_ + ".png";
    cv::Mat img2_ = cv::imread(filepath);
    if (img1_.empty() || img2_.empty())//读取失败时
    {
        cout << "[ERROR]Could not open or find the image,please check the image path" <<endl;
        return;
    }
    resize(img1_, img1_, cv::Size(621, 188));
    resize(img2_, img2_, cv::Size(621, 188));
    cv::Mat img1, img2;

    cv::cvtColor(img1_, img1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2_, img2, cv::COLOR_BGR2GRAY);
    
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,48,5,10,20);
    sgbm->setPreFilterCap(31);
    sgbm->setUniquenessRatio(5);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->compute(img1,img2,DispOpenCV);
    DispOpenCV.convertTo(DispOpenCV, CV_32F, 1.0/16);
    for(int i=0;i<DispOpenCV.rows;++i){
        for(int j=0;j<DispOpenCV.cols;++j){
            if(DispOpenCV.at<float>(i,j)<0)
                DispOpenCV.at<float>(i,j)=0;
        }
    }
    cv::Mat DispImg = cv::Mat::zeros(img1.rows, img1.cols, CV_32FC1);
    int Rows = img1.rows;
    int Cols = img1.cols;
    vector<thread> threads;
    for (int row =5; row <= ((Rows - 5) - 1)/2; ++row) {
        threads.push_back(thread(GetDisparityMultiThread, std::ref(DispImg), std::ref(img1), std::ref(img2), row, Cols));//task.detach();
    }
    for (auto &&i:threads) {
        i.join();
    }
    for(int i=0;i<((DispOpenCV.rows-PATCHRADIUS)-1)/2;++i){
        for(int j=0;j<DispOpenCV.cols;++j){
            if(DispImg.at<float>(i,j)==0)
                DispOpenCV.at<float>(i,j)=0;
        }
    }
#ifdef DEBUG_INFORMATION
    cv::Mat Display;
    double normmax1;
    minMaxLoc(DispOpenCV,NULL,&normmax1);
    DispOpenCV.convertTo(Display,CV_8UC1,255.0/normmax1,0);
    cout<<"[DEBUG INFORMATION] Processing image:"+num_+".png"<<endl;
    // cv::imwrite("disparity.bmp", Display);
    // cv::namedWindow("disparity map",0);
    // cv::imshow("disparity map",Display);
    // cv::waitKey(1);
#endif
}

void Recover3D(vector<cv::Point3f> &Point3D,cv::Mat &DispImg,cv::Mat &K,cv::Mat &Pose,int index,vector<vector<float>> &AllIntensity){
    char num[6];
    sprintf(num,"%06d",index);
    string num_=num;

    string filepath="../DataSet/image_2/"+num_+".png";
    cv::Mat img = cv::imread(filepath);
    if (img.empty() )//读取失败时
    {
        cout << "[ERROR]Could not open or find the image,please check the image path" <<endl;
        return;
    }
    resize(img, img, cv::Size(621, 188));

    for (int i = 0; i < DispImg.rows; ++i) {
        for (int j = 0; j < DispImg.cols; ++j) {
            if (DispImg.at<float>(i, j) == 0) { continue; }
            ///-------------------------------------------
            cv::Mat lamda=LinearLeastSquares(DispImg,K,i,j);
            ///-------------------------------------------
            cv::Mat ptc = lamda.at<float>(0, 0) * K.inv()*(cv::Mat_<float>(3, 1) << j, i, 1.0);
            cv::Mat wtc = (cv::Mat_<float>(3, 3) << 0, -1, 0, 0, 0, -1, 1, 0, 0);//改变xyz轴朝向
            cv::Mat pt = wtc.inv() * ptc;
            if (pt.at<float>(0, 0) > 20 || pt.at<float>(0, 0) < 7) { continue; }
            if (pt.at<float>(1, 0) > 10 || pt.at<float>(1, 0) < -10) { continue; }
            if (pt.at<float>(2, 0) > 10 || pt.at<float>(2, 0) < -10) { continue; }
            cv::Point3f temp;
            temp.x = ptc.at<float>(0, 0);
            temp.y = ptc.at<float>(1, 0);
            temp.z = ptc.at<float>(2, 0);
            Point3D.push_back(temp);

            vector<float> ColorPixel(3,0.0);
            ColorPixel[0]=img.at<cv::Vec3b>(i, j)[0];
            ColorPixel[1]=img.at<cv::Vec3b>(i, j)[1];
            ColorPixel[2]=img.at<cv::Vec3b>(i, j)[2];
            AllIntensity.push_back(ColorPixel);
        }
    }
}

void Viusalization(vector<cv::Point3f> AllPoints,vector<vector<float>> AllIntensity){
    std::ofstream fout1("points.ply");
    fout1<<"ply"<<"\n"<<"format ascii 1.0"<<"\n"<<"element vertex "<<AllPoints.size()<<"\n"<<
    "property double x"<<"\n"<<
    "property double y"<<"\n"<<
    "property double z"<<"\n"<<
    "property uchar red"<<"\n"<<
    "property uchar green"<<"\n"<<
    "property uchar blue"<<"\n"<<
    "end_header"<<"\n";
    fout1.close();
    for(int i=0;i<AllPoints.size();++i){
        std::ofstream fout("points.ply",std::ios::app);
        fout<<AllPoints[i].x<<"\ ";
        fout<<AllPoints[i].y<<"\ ";
        fout<<AllPoints[i].z<<"\ ";
        fout<<AllIntensity[i][2]<<"\ ";
        fout<<AllIntensity[i][1]<<"\ ";
        fout<<AllIntensity[i][0]<<"\n";
        fout.close();
    }
}

void TEST_LINEARLEASTSQUARES(){
    cv::Mat K=(cv::Mat_<float>(3, 3) << 359.42801, 0, 303.59641,0, 359.42801, 92.607849,0, 0, 1);
    cv::Mat Disp=(cv::Mat_<float>(1, 1) << 17.3125);
    cv::Mat lamda_real=(cv::Mat_<float>(2, 1) << 11.211838,11.211807);
    cv::Mat lamda=LinearLeastSquares(Disp,K,0,0);
    if(abs(lamda.at<float>(0,0)-lamda_real.at<float>(0,0))<0.01&&abs(lamda.at<float>(1,0)-lamda_real.at<float>(1,0))<0.01){
        cout<<"[TEST INFORMATION] Your implementation of LinearLeastSquares passes the test!"<<endl;
    }
    else{
        cout<<"[ERROR]"<<endl;
        cout<<"the expected result:"<<lamda_real<<endl;
        cout<<"your result:"<<lamda<<endl;
        cout<<"you may check your implementation of LinearLeastSquares"<<endl;
    }
}
void TEST_COORDINATETRANSFORM(){
    cv::Point3f temp;
    temp.x=-7.4738636;
    temp.y=-2.73280048;
    temp.z=11.2118378;
    vector<cv::Point3f> Point3D;
    Point3D.push_back(temp);

    temp.x=-7.4738636;
    temp.y=-2.73280048;
    temp.z=11.2118368;
    vector<cv::Point3f> wPoint3D_real;
    wPoint3D_real.push_back(temp);
    cv::Mat Pose=(cv::Mat_<float>(1, 12) << 1, 9.0436799e-12, 2.3268091e-11, 5.5511151e-17, 9.0436833e-12,
            1, 2.3923699e-10, 3.3306691e-16, 2.3268099e-11, 2.3923699e-10, 0.99999988, -4.4408921e-16);
    vector<cv::Point3f> wPoint3D=CameraToWorld(0,Point3D,Pose);
    if(abs(wPoint3D_real[0].x-wPoint3D[0].x)<0.01&&
            abs(wPoint3D_real[0].y-wPoint3D[0].y)<0.01&&
            abs(wPoint3D_real[0].z-wPoint3D[0].z)<0.01){
        cout<<"[TEST INFORMATION] Your implementation of CameraToWorld passes the test!"<<endl;
    }
    else{
        cout<<"[ERROR]"<<endl;
        cout<<"the expected result:"<<wPoint3D_real<<endl;
        cout<<"your result:"<<wPoint3D<<endl;
        cout<<"you may check your implementation of CameraToWorld"<<endl;
    }
}