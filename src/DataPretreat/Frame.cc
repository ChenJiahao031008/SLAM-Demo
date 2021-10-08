/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-08 11:16:02
 * @LastEditTime: 2021-10-08 20:40:12
 * @LastEditors: Chen Jiahao
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/src/DataPretreat/Frame.cc
 */
#include "DataPretreat/Frame.h"

Frame::Frame()
{}

Frame::Frame(const cv::Mat &imgRGBL_, const cv::Mat &imgRGBR_, Config &config_)
    : imgRGBL(imgRGBL_.clone()), imgRGBR(imgRGBR_.clone())
{
    if (config_.app.Resize == true)
        cv::resize(imgRGBL, imgResizeL, cv::Size(640, 480));
    else
        imgResizeL = imgRGBL;

    cvtColor(imgResizeL, imgGrayL, cv::COLOR_BGR2GRAY);

    FeatureManager fm(config_.app);
    fm.FeatureExtraction(imgGrayL, KeyPoints, Descriptors);

    BuildDepthMap(imgRGBR_, config_);
}

void Frame::BuildDepthMap(const cv::Mat &imageR_, Config &config_)
{
    DepthMap dm;

    if (config_.app.Mode == "Stereo")
    {
        if (config_.app.Resize == true)
            resize(imageR_, imgResizeR, cv::Size(640, 480));
        else
            imgResizeR = imageR_;

        cv::cvtColor(imgResizeR, imgGrayR, cv::COLOR_BGR2GRAY);
        dm.StereoDepthBuilder(imgGrayL, imgGrayR, config_);

    }else if (config_.app.Mode == "RGBD"){
        dm.RGBDDepthBuilder(imgDepth, config_);
    }else{
        std::cout << "[ERROR] We Only Support Stereo or RGBD." << std::endl;
    }

    imgDepth = dm.GetDepthMap();
}

void Frame::GetKPDes(std::vector<cv::KeyPoint> &KeyPoints_, cv::Mat &Descriptors_)
{
    KeyPoints_ = KeyPoints;
    Descriptors_ = Descriptors.clone();
}

cv::Mat Frame::GetDepthMap()
{
    return imgDepth;
}

cv::Mat Frame::GetIMGLeft()
{
    return imgRGBL;
}

Frame::~Frame()
{
}
