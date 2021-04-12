/*
 * @Author: your name
 * @Date: 2021-03-21 19:17:57
 * @LastEditTime: 2021-04-07 10:05:02
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /极线可视化/test.cpp
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cassert>
#include "FeatureMather.h"
#include "EpipolarLine.h"
#include "Config.h"
#include "DepthMap.h"
#include "PoseSolver.h"
#include "Optimization.h"

using namespace std;
using namespace cv;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char const *argv[])
{

    // ========= Part1： 读取参数配置文件，读取图片及预处理 ============= //
    if ( argc != 3){
        cerr << "[ERRO] Please check argv! " << endl;
        cerr << "path_to_setting path_to_sequence" << endl;
        return -1;
    }

    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << argv[1] << endl;
       exit(-1);
    }

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]),vstrImageLeft,vstrImageRight,vTimestamps);
    const int nImages = vTimestamps.size();

    cout << "Total have " << nImages << " images in the sequence" << endl;

    Config conf(fsSettings);
    for(int i=0; i<nImages; i++)
    {

    cv::Mat imgBGR_0 = cv::imread(vstrImageLeft[i],CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat imgBGR_1 = cv::imread(vstrImageRight[i],CV_LOAD_IMAGE_UNCHANGED);
    if (imgBGR_0.empty() || imgBGR_1.empty()){
        cerr << "[ERRO] Please check the path of images!" << endl;
        return -1;
    }

    cv::Mat imgRize_0, imgRize_1, imgGray_0, imgGray_1;
    if (conf.app.Resize == true){
        resize(imgBGR_0, imgRize_0, cv::Size(640,480));
        resize(imgBGR_1, imgRize_1, cv::Size(640,480));
    }else{
        imgRize_0 = imgBGR_0;
        imgRize_1 = imgBGR_1;
    }

    cvtColor(imgRize_0,imgGray_0,COLOR_BGR2GRAY);
    cvtColor(imgRize_1,imgGray_1,COLOR_BGR2GRAY);
     // CHECK POINT 1
//     imshow("input_image_1",imgRize_0);
//     waitKey(0);
//     imshow("input_image_2",imgRize_1);
//     waitKey(0);
//
    // ============== Part2： 特征点检测及匹配 =============== //
    std::vector<KeyPoint> Kp1,Kp2;
    std::vector<DMatch> matches;
    FeatureMatcher fm(conf.app,conf.ip, imgGray_0, imgGray_1);
    fm.FeatureExtraction(matches, Kp1, Kp2);
    // ================= Part3： 深度图构建 ================== //
    DepthMap dm(imgGray_0, imgGray_1);
    if (conf.app.Stereo==true){
        conf.ss.img_0_left = conf.app.img_0;
        conf.ss.img_1_left = conf.app.img_1;

        cv::Mat imgBGR_0_r = cv::imread(conf.ss.img_0_right);
        cv::Mat imgBGR_1_r = cv::imread(conf.ss.img_1_right);
        cv::Mat imgRize_0_r, imgRize_1_r, imgGray_0_r, imgGray_1_r;
        if (conf.app.Resize == true){
            resize(imgBGR_0_r, imgRize_0_r, cv::Size(640,480));
            resize(imgBGR_1_r, imgRize_1_r, cv::Size(640,480));
        }else{
            imgRize_0_r = imgBGR_0_r;
            imgRize_1_r = imgBGR_1_r;
        }
        cvtColor(imgRize_0_r,imgGray_0_r,COLOR_BGR2GRAY);
        cvtColor(imgRize_1_r,imgGray_1_r,COLOR_BGR2GRAY);

        cv::Mat imgGray_0_l = imgGray_0;
        cv::Mat imgGray_1_l = imgGray_1;

        dm.StereoDepthBuilder(imgGray_0_r, imgGray_1_r, conf);
    }

    // cv::Mat imgDepth_0_left = dm.GetDepthMap_0();
    // cv::Mat imgDepth_1_left = dm.GetDepthMap_1();
    std::cout << "[INFO] DepthMap Build Finished! " << std::endl;

//    // ========= Part4： 位姿估计，确定相机相对运动及坐标 ============= //
//    std::vector<cv::Point3f> PixelPoint3fVec_0, PixelPoint3fVec_1;
//    for (size_t i(0); i< matches.size(); ++i){
//        cv::Point2f tmpPoint_0 = Kp1[matches[i].queryIdx].pt;
//        cv::Point2f tmpPoint_1 = Kp2[matches[i].trainIdx].pt;
//        double depth_0 = dm.GetDepth(tmpPoint_0,0);
//        double depth_1 = dm.GetDepth(tmpPoint_1,1);
//        if (depth_0<=0 || depth_1 <=0) continue;
//        PixelPoint3fVec_0.emplace_back(tmpPoint_0.x, tmpPoint_0.y, depth_0);
//        PixelPoint3fVec_1.emplace_back(tmpPoint_1.x, tmpPoint_1.y, depth_1);
//    }
//
//    PoseSolver ps(PixelPoint3fVec_0, PixelPoint3fVec_1, conf, 1);
//    ps.ComputePnP();
//
//
//    EpipolarLine el;
    // Optimization opt;

    }

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    string s;
    while(!fTimes.eof())
    {
        getline(fTimes,s);
        stringstream ss;
        ss << s;
        double t;
        ss >> t;
        vTimestamps.push_back(t);
    }

    const int nTimes = vTimestamps.size();
    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft.push_back(strPrefixLeft + ss.str() + ".png");
        vstrImageRight.push_back(strPrefixRight + ss.str() + ".png");
    }
}