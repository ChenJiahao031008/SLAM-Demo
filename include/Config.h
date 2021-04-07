/*
 * @Author: your name
 * @Date: 2021-03-26 11:25:51
 * @LastEditTime: 2021-04-06 21:51:27
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /极线可视化/include/config.h
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>

class Config
{
public:
    struct AppSettings
    {
        std::string img_0;
        std::string img_1;
        int ORB_Features;
        int SIFT_Features;
        int SURF_Features;
        int Resize;
        int Stereo;
    };

    struct StereoSetting
    {
        std::string img_0_left;
        std::string img_1_left;
        std::string img_0_right;
        std::string img_1_right;
        int MatchingMethod;
        float baseline;
        float DepthTH;
    };

    struct InternalParameters
    {
        float fx, fy, cx, cy;
        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        cv::Mat DistCoef = cv::Mat::zeros(4,1,CV_32F);
    };

    struct OptimizationConfig
    {
        int maxRansacIter;
        double ErrorTH;
    };



public:
    cv::FileStorage SettingsFile;

    AppSettings app;
    StereoSetting ss;
    InternalParameters ip;
    OptimizationConfig oc;

public:
    Config(cv::FileStorage &fsSettings):SettingsFile(fsSettings)
    {
        AppSettingsInit();

        InternalParametersInit();
        OptimizationConfigInit();

        if (app.Stereo)
            StereoSettingInit();

    };

    void AppSettingsInit(){
        app.img_0 = static_cast<std::string>(SettingsFile["Image.left.first"]);
        app.img_1 = static_cast<std::string>(SettingsFile["Image.left.second"]);
        app.ORB_Features  = SettingsFile["Image.ORB_Features"];
        app.SIFT_Features = SettingsFile["Image.SIFT_Features"];
        app.SURF_Features = SettingsFile["Image.SURF_Features"];
        app.Resize = SettingsFile["Image.Resize"];
        app.Stereo = SettingsFile["Image.Stereo"];
    };

    void StereoSettingInit(){
        ss.baseline = SettingsFile["Stereo.bf"];
        ss.DepthTH = SettingsFile["Stereo.th"];
        ss.img_0_right = static_cast<std::string>(SettingsFile["Stereo.right.first"]);
        ss.img_1_right = static_cast<std::string>(SettingsFile["Stereo.right.second"]);
        ss.MatchingMethod = SettingsFile["Stereo.MatchingMethod"];
    };

    void InternalParametersInit(){
        ip.fx = SettingsFile["Camera.fx"];
        ip.fy = SettingsFile["Camera.fy"];
        ip.cx = SettingsFile["Camera.cx"];
        ip.cy = SettingsFile["Camera.cy"];
        ip.K.at<float>(0,0) = ip.fx;
        ip.K.at<float>(1,1) = ip.fy;
        ip.K.at<float>(0,2) = ip.cx;
        ip.K.at<float>(1,2) = ip.cy;
        ip.DistCoef.at<float>(0) = SettingsFile["Camera.k1"];
        ip.DistCoef.at<float>(1) = SettingsFile["Camera.k2"];
        ip.DistCoef.at<float>(2) = SettingsFile["Camera.p1"];
        ip.DistCoef.at<float>(3) = SettingsFile["Camera.p2"];
    };

    void OptimizationConfigInit(){
        oc.maxRansacIter = SettingsFile["Opts.maxRansacIter"];
        oc.ErrorTH = SettingsFile["Opts.ErrorTH"];
    }

};

#endif //CONFIG_H

