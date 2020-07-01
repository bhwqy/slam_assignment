#include "CalculateDisp.h"
extern int PATCHRADIUS;
extern int MINDISP;
extern int MAXDISP;
extern int patchsize;
/*calculate the disparity for each pixel in the left image.Read the PDF tutorial for more information
 *INPUT:
 * left_img: image catched by left camera
 * right_img:image catched by right camera
 * row:      the pixel's location(row) in the left image
 * col:      the pixel's location(col) in the left image
 *OUTPUT:
 * D:        the disparity of this pixel
 */
int CalculateDisp(const cv::Mat left_img,const cv::Mat right_img, int row, int col){
    int D;
    vector<float> ssd;
    int rows = left_img.rows, cols = left_img.cols;
    for (int k = col - MAXDISP; k <= col - MINDISP; ++k) {
        float ans = 0.0;
        for (int i = -PATCHRADIUS; i <= PATCHRADIUS; ++i) {
            for (int j = -PATCHRADIUS; j <= PATCHRADIUS; ++j) {
                if (row + i < 0 || row + i >= rows || col + j >= cols || k + j < 0)
                    continue;
                float diff = static_cast<float>(left_img.at<uchar>(row + i, col + j))
                    - static_cast<float>(right_img.at<uchar>(row + i, k + j));
                ans += diff * diff;
            }
        }
        ssd.push_back(ans);
    }
    float ssd_min = FLT_MAX;
    int ssd_min_id = -1;
    for (int i = 0; i < ssd.size(); ++i) {
        if (ssd_min > ssd[i]) {
            ssd_min = ssd[i];
            ssd_min_id = i;
        }
    }
    if (ssd_min_id == col - MAXDISP || ssd_min_id == col - MINDISP)
        return -1;
    int cnt = 0;
    for (auto each : ssd)
        if (each < 1.5 * ssd_min)
            cnt++;
    if (cnt >= 2) 
        return -1;
    D = 50 - ssd_min_id;
    return D;
}
