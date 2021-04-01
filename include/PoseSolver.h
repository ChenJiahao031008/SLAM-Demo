/*
 * @Author: your name
 * @Date: 2021-03-28 17:41:54
 * @LastEditTime: 2021-04-01 19:33:58
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /极线可视化/include/PoseSolver.h
 */
#ifndef POSESOLVER_H
#define POSESOLVER_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

// #include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "Config.h"

class PoseSolver
{
private:
    std::vector<cv::Point3f> PointsInPixelVec_0, PointsInPixelVec_1;
    std::vector<cv::Point3f> PointsInWorldVec_0, PointsInWorldVec_1;
    Config setting;

    Eigen::Matrix4f Pose_0, Pose_1;
    Eigen::Vector3f CamOrientation_0, CamOrientation_1;
    Eigen::Matrix4f T12;

    std::map<int, std::vector<cv::Point3f>* > PixelPointsDict;
    std::map<int, std::vector<cv::Point3f>* > WorldPointsDict;

    Eigen::Matrix3f eigenK;

public:
    PoseSolver(std::vector<cv::Point3f> &coord_0, std::vector<cv::Point3f> &coord_1, Config &config);

    ~PoseSolver();

    void SetOrigin(const int &flag);

    void ComputePnP();

    void KneipPnP(const int &a, const int &b, const int &c, const int &d);

    void solve_quartic_roots(Eigen::Matrix<float,5,1> const& factors, std::vector<double> &real_roots);

    double ReProjectError(const int &i, Eigen::Matrix4f &T);

    void CheckReProjrctError();

    void Project(Eigen::Vector4f &CamCoord, Eigen::Vector3f & PixelCoord);

    void Unproject(const int &PoseId, const Eigen::Matrix4f &T);

};

#endif // POSESOLVER_H