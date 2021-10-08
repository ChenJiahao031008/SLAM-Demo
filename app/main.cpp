#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cassert>
#include <chrono>
#include "DataPretreat/Config.h"
#include "DataPretreat/DepthMap.h"
#include "DataPretreat/Frame.h"

#include "FrontEnd/FeatureMather.h"
#include "FrontEnd/PoseSolver.h"
#include "FrontEnd/FeatureManager.h"

#include "BackEnd/Optimization.h"

using namespace std;
using namespace cv;

void LoadImagesStereo(const string &strPathToSequence, vector<string> &vstrImageLeft,
                      vector<string> &vstrImageRight, vector<string> &vTimestamps);

void LoadImagesRGBD(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<string> &vTimestamps);

int main(int argc, char const *argv[])
{

    // Part 1：读取参数配置文件，读取图片及预处理
    if ( argc != 2){
        cerr << "[ERROR] Please check argv! " << endl;
        return -1;
    }


    // Step 1_1 读取参数文件
    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "[ERROR] Failed to open settings file at: " << argv[1] << endl;
        exit(-1);
    }
    Config conf(fsSettings);
    cout << "[INFO] Loading the parameter files...." << endl;


    // Step 1_2 读取图像序列地址
    vector<string> vstrImageLeft, vstrImageRight, vTimestamps;
    if (conf.app.img_dataset == "KITTI")
        LoadImagesStereo(conf.app.img_path, vstrImageLeft, vstrImageRight, vTimestamps);
    else if (conf.app.img_dataset == "TUM")
        LoadImagesRGBD(conf.app.img_path, vstrImageLeft, vstrImageRight, vTimestamps);
    else
        cout << "[ERROR] Only Support KITTI and TUM! " << endl;
    const int nImages = vstrImageLeft.size();

    cout << "Total have " << nImages << " images in the sequence" << endl;

    cv::Mat imgRGBL_0, imgRGBR_0, imgDepth_0;
    cv::Mat imgRGBL_1, imgRGBR_1, imgDepth_1;
    imgRGBL_0 = cv::imread(vstrImageLeft[0], CV_LOAD_IMAGE_UNCHANGED);
    imgRGBR_0 = cv::imread(vstrImageRight[0], CV_LOAD_IMAGE_UNCHANGED);
    Frame preFrame(imgRGBL_0, conf);

    // Part 2：对每相邻两帧图像进行匹配和位姿计算
    for(size_t i=1; i<nImages; i++)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Step 2_1 读取图像并提取特征
        imgRGBL_1 = cv::imread(vstrImageLeft[i], CV_LOAD_IMAGE_UNCHANGED);
        imgRGBR_1 = cv::imread(vstrImageRight[i], CV_LOAD_IMAGE_UNCHANGED);
        if (imgRGBL_0.empty() || imgRGBR_1.empty())
        {
            cerr << "[ERROR] Please check the path of images!" << endl;
            return -1;
        }

        cout << "===>> Current Frame: " << vTimestamps[i] << endl;

        Frame curFrame(imgRGBL_1, conf);
        imgDepth_0 = preFrame.BuildDepthMap(imgRGBR_0, conf);
        imgDepth_1 = curFrame.BuildDepthMap(imgRGBR_1, conf);
        std::cout << "[INFO] DepthMap Build Finished! " << std::endl;

        // Step 2_2 特征匹配
        std::vector<DMatch> matches;
        std::vector<cv::KeyPoint> KPs1, KPs2;
        cv::Mat Des1, Des2;
        preFrame.GetKPDes(KPs1, Des1);
        curFrame.GetKPDes(KPs2, Des2);

        FeatureManager fm(conf.app);
        fm.FeatureMatch(Des1, Des2, matches);

        preFrame = curFrame;

        // Step 2_3_3 计算三维空间点
        std::vector<cv::Point3f> PixelPoint3fVec_0, PixelPoint3fVec_1;
        std::vector<cv::KeyPoint> KPs1_Opt, KPs2_Opt;
        for (size_t i(0); i< matches.size(); ++i){
            cv::Point2f tmpPoint_0 = KPs1[matches[i].queryIdx].pt;
            cv::Point2f tmpPoint_1 = KPs2[matches[i].trainIdx].pt;
            KPs1_Opt.emplace_back(KPs1[matches[i].queryIdx]);
            KPs2_Opt.emplace_back(KPs2[matches[i].queryIdx]);
            double depth_0 = imgDepth_0.ptr<float>((int)tmpPoint_0.y)[(int)tmpPoint_0.x];
            double depth_1 = imgDepth_1.ptr<float>((int)tmpPoint_1.y)[(int)tmpPoint_1.x];
            if (depth_0<=0 || depth_1 <=0) continue;
            PixelPoint3fVec_0.emplace_back(tmpPoint_0.x, tmpPoint_0.y, depth_0);
            PixelPoint3fVec_1.emplace_back(tmpPoint_1.x, tmpPoint_1.y, depth_1);
        }
        // CHECK POINT 2
        cv::Mat KeyPointShow;
        // drawKeypoints(imgRize_0, KPs1_Opt, KeyPointShow, cv::Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
        drawKeypoints(imgRGBR_1, KPs2_Opt, KeyPointShow, cv::Scalar(0, 255, 0), DrawMatchesFlags::DEFAULT);
        imshow("KeyPointShow",KeyPointShow);
        if (PixelPoint3fVec_0.size() <6 || PixelPoint3fVec_1.size() <6){
            cout << "[WARRING] Mathes less than 6 pairs." << endl;
            waitKey(0);
            continue;
        }

        // Step 2_4 P3P位姿求解及优化
        PoseSolver ps(PixelPoint3fVec_0, PixelPoint3fVec_1, conf, 1);
        ps.ComputePnP();

        waitKey(1);


        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "[INFO] costs time: " << time_used.count() << " seconds." << std::endl;
    }

    return 0;
}

void LoadImagesStereo(const string &strPathToSequence, vector<string> &vstrImageLeft,
                      vector<string> &vstrImageRight, vector<string> &vTimestamps)
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
        double nt;
        ss >> nt;
        std::string st = to_string(nt);
        vTimestamps.push_back(st);
    }

    const int nTimes = 20;
    string strPrefixLeft = strPathToSequence + "/image_2/";
    string strPrefixRight = strPathToSequence + "/image_3/";
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft.push_back(strPrefixLeft + ss.str() + ".png");
        vstrImageRight.push_back(strPrefixRight + ss.str() + ".png");
    }
    cout << vstrImageLeft.size() << endl;
}

void LoadImagesRGBD(const string &strPathToSequence, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<string> &vTimestamps)
{
    ifstream fAssociation;
    const string associationFile = strPathToSequence + "associate.txt";
    fAssociation.open(associationFile.c_str());
    cout << "[LOAD] Open Association File ..." << endl;
    while (!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            string st, sRGB, sD;
            ss >> st;
            vTimestamps.emplace_back(st);
            ss >> sRGB;
            vstrImageFilenamesRGB.emplace_back(strPathToSequence + sRGB);
            ss >> st;
            ss >> sD;
            vstrImageFilenamesD.emplace_back(strPathToSequence + sD);
        }
    }
}
