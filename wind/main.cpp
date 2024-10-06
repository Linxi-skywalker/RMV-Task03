#include "windmill.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ceres/ceres.h>
#include <vector>


using namespace std;
using namespace cv;


struct AngleCostFunctor
{
    AngleCostFunctor(double t, double angle) : t_(t), angle_(angle) {}

    template <typename T>
    bool operator()(const T *const A0, const T *const A, const T *const w, const T *const phi, T *residual) const
    {
        residual[0] = angle_ - cos(A0[0] * t_ + A[0] / w[0] * (cos(phi[0] + 1.57) - cos(w[0] * t_ + phi[0] + 1.57)));
        return true;
    }

private:
    const double t_;
    const double angle_;
};
inline bool isWithRange(double A0, double A, double w) {
    // 定义参数的边界值
    const double w_min = 1.7898;
    const double w_max = 1.9782;
    const double A_min = 0.74575;
    const double A_max = 0.82425;
    const double A0_min = 1.23975;
    const double A0_max = 1.37025;

    // 检查参数是否在指定的范围内
    return 
           w >= w_min && w <= w_max &&
           A >= A_min && A <= A_max &&
           A0 >= A0_min && A0 <= A0_max;
}
int main() {
	double t_sum = 0;
    	const int N = 10;
    	for (int num = 0; num < N; num++)
    {
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(
    	std::chrono::system_clock::now().time_since_epoch());
    	WINDMILL::WindMill wm(t.count());
    	cv::Mat src;
    	
    	ceres::Problem problem;
        double A0 = 0.305, A = 1.785, w = 0.884, phi = 1.24;
        double A0_i = 1.305, A_i = 0.785, omega_i = 1.884, phi_i = 0.24;
        double t_start = (double)t.count();
        int64 start_time = getTickCount();
        
    	while (1) {
    	        
        	t = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());
        	src = wm.getMat((double)t.count() /1);

        //==========================代码区========================//

        
        Mat gray;
        cvtColor(src, gray, COLOR_BGR2GRAY);                     // 转换为灰度图
        Mat binary;
        threshold(gray, binary, 75, 255, THRESH_BINARY);         // 二值化处理

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        int Ri = -1, hi = -1;
        for (size_t i = 0; i < contours.size(); ++i) {
    
            if (Ri != -1 && hi != -1) {
                break;                                           // 如果已经找到了Ri和hi，则提前退出循环
            }
            if (hierarchy[i][3] == -1) {
                double area = contourArea(contours[i]);           // 检查当前轮廓是否是顶层轮廓
                if (area < 500) {
                    Ri = area < 250 ? i : Ri;                     // 识别R标    如果面积小于250，设置Ri；否则保持Ri不变
                } 
                else if (area < 5000) {
                    hi = hierarchy[i][2];                         // 识别不同的扇叶，即为锤子  并找到锤头的轮廓
                }
            }
        }

       	Moments momR = moments(contours[Ri], false);
        int cX = static_cast<int>(momR.m10 / momR.m00);
        int cY = static_cast<int>(momR.m01 / momR.m00);
        Point R_C(int(momR.m10 / momR.m00), int(momR.m01 / momR.m00));
        circle(src, Point(cX, cY), 5, Scalar(255, 0, 0), -1);     //计算R标的中心点位置并且画上圆点

        Moments momH = moments(contours[hi], false);
        int cX2 = static_cast<int>(momH.m10 / momH.m00);
        int cY2 = static_cast<int>(momH.m01 / momH.m00);
        Point H_C2(int(momH.m10 / momH.m00), int(momH.m01 / momH.m00));
        circle(src, Point(cX2, cY2), 5, Scalar(255, 0, 0), -1);   //计算锤头部分中心点位置并且画上圆点
        
       // 计算从R标中心到锤头中心的向量
Point2d vectorPR(H_C2 - R_C);
// 计算单位向量
Point2d normvectorPR = vectorPR / norm(vectorPR);


        
        double t_1=(double)t.count();
        double x_data = (t_1 - t_start) / 1000;
       double y_data = normvectorPR.x;  // 提取x坐标
        
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<AngleCostFunctor, 1, 1, 1, 1, 1>(new AngleCostFunctor(x_data, y_data)), NULL, &A0, &A, &w, &phi);

            ceres::Solver::Options options;
            options.max_num_iterations = 20;
            options.linear_solver_type = ceres::DENSE_QR;

            problem.SetParameterLowerBound(&A0, 0, 0.5);
            problem.SetParameterUpperBound(&A0, 0, 1.5);
            problem.SetParameterLowerBound(&A, 0, 0.5);
            problem.SetParameterUpperBound(&A, 0, 1.0);
            problem.SetParameterLowerBound(&w, 0, 1.0);
            problem.SetParameterUpperBound(&w, 0, 2.0);
            problem.SetParameterLowerBound(&phi, 0, 0.24);
            problem.SetParameterUpperBound(&phi, 0, 1.25);

            ceres::Solver::Summary summary;
            Solve(options, &problem, &summary);
            
            
            if (isWithRange(A0, A, w))
            {
                
                int64 end_time = getTickCount();
                t_sum += (end_time - start_time) / getTickFrequency();
                break;
            }
            
            
            

        //=======================================================//

        //imshow("windmill", src);

        waitKey(1);
    }
    }
   std::cout<<t_sum/N<<std::endl;  
}
