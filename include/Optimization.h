/*
 * @Author: your name
 * @Date: 2021-04-02 10:37:55
 * @LastEditTime: 2021-04-07 09:14:57
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/include/Optimization.h
 */

#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Config.h"

class Optimization
{
private:
    Config setting;
public:
    Optimization(Config &config);
    ~Optimization();

    // template<T, N>
    // void LeastSquare(std::vector<T> &Observation, N &Result);

    void Kneip_Ransac(std::vector<cv::Point3f> &PointsInWorldVec_0, std::vector<cv::Point3f> &PointsInPixelVec_1, std::vector<int> &Interior, Eigen::Matrix4f &Pose);
};


#endif //OPTIMIZATION_H

