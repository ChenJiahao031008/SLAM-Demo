/*
 * @Author: your name
 * @Date: 2021-03-26 14:55:09
 * @LastEditTime: 2021-04-01 18:29:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /极线可视化/include/DepthMap.h
 */
#ifndef DEPTHMAP_H
#define DEPTHMAP_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include "Config.h"

class DepthMap
{
private:
    cv::Mat Frame_0_left, Frame_1_left, Frame_0_right,Frame_1_right;
    cv::Mat DepthImage_0_left, DepthImage_1_left;
public:
    DepthMap(cv::Mat &img_0, cv::Mat &img_1);

    void StereoDepthBuilder(cv::Mat &img_0, cv::Mat& img_1, Config& config);

    void insertDepth32f(cv::Mat& depth);

    void CheckDepthMap(cv::Mat& depth, cv::Mat &depth_show);

    cv::Mat SGBM(Config &config, cv::Mat &img_left, cv::Mat &img_right);

    cv::Mat GetDepthMap_0(){ return DepthImage_0_left; };

    cv::Mat GetDepthMap_1(){ return DepthImage_1_left; };

    double GetDepth(cv::Point2f &coordinate, const int &flag);

};

void Mouse_Callback(int event,int x,int y,int flags,void *param);



#endif //DEPTHMAP_H