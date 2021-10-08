/*
 * @Author: Chen Jiahao
 * @Date: 2021-10-08 14:09:37
 * @LastEditors: Chen Jiahao
 * @LastEditTime: 2021-10-08 21:27:34
 * @Description: file content
 * @FilePath: /SLAM-Demo/src/FrontEnd/Tracking.cc
 */

#include "FrontEnd/Tracking.h"

Tracking::Tracking(/* args */)
{
}

void Tracking::RunTracking(Frame &preFrame, Frame &curFrame, Config &conf)
{
    // Step 1 特征获取及匹配
    std::vector<cv::KeyPoint> KPs1, KPs2;
    cv::Mat Des1, Des2;
    preFrame.GetKPDes(KPs1, Des1);
    curFrame.GetKPDes(KPs2, Des2);

    FeatureManager fm(conf.app);
    std::vector<cv::DMatch> matches;
    fm.FeatureMatch(Des1, Des2, matches);

    // Step 2 位置和深度融合
    cv::Mat preDepth = preFrame.GetDepthMap();
    cv::Mat curDepth = curFrame.GetDepthMap();
    std::vector<cv::Point3f> prePixelPoints, curPixelPoints;
    FusionPosAndDepth(preDepth, curDepth, KPs1, KPs2, Des1, Des2, matches, prePixelPoints, curPixelPoints);

    // Step 3 P3P位姿求解及优化
    PoseSolver ps(prePixelPoints, curPixelPoints, conf, 1);
    ps.ComputePnP();

    // _____CHECK POINT 2_____
    {
        cv::Mat KeyPointShow;
        cv::Mat curimgRGB = curFrame.GetIMGLeft();
        cv::drawKeypoints(curimgRGB, KPs2, KeyPointShow, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
        imshow("KeyPointShow", KeyPointShow);
        if (prePixelPoints.size() < 6 || curPixelPoints.size() < 6)
        {
            std::cout << "[WARRING] Mathes less than 6 pairs." << std::endl;
            cv::waitKey(0);
            return;
        }
    }

}

void Tracking::FusionPosAndDepth(
    const cv::Mat &preDepth, const cv::Mat &curDepth,
    const std::vector<cv::KeyPoint> &KPs1, const std::vector<cv::KeyPoint> &KPs2,
    const cv::Mat &Des1, const cv::Mat &Des2,
    const std::vector<cv::DMatch> matches,
    std::vector<cv::Point3f> &prePixelPoints, std::vector<cv::Point3f> &curPixelPoints)
{
    std::vector<cv::KeyPoint> KPs1_Opt, KPs2_Opt;
    for (size_t i(0); i < matches.size(); ++i)
    {
        cv::Point2f prePoint = KPs1[matches[i].queryIdx].pt;
        cv::Point2f curPoint = KPs2[matches[i].trainIdx].pt;
        KPs1_Opt.emplace_back(KPs1[matches[i].queryIdx]);
        KPs2_Opt.emplace_back(KPs2[matches[i].queryIdx]);
        double preDepthPoint = preDepth.ptr<float>((int)prePoint.y)[(int)prePoint.x];
        double curDepthPoint = curDepth.ptr<float>((int)curPoint.y)[(int)curPoint.x];
        if (preDepthPoint <= 0 || curDepthPoint <= 0)
            continue;
        prePixelPoints.emplace_back(prePoint.x, prePoint.y, preDepthPoint);
        curPixelPoints.emplace_back(curPoint.x, curPoint.y, curDepthPoint);
    }

}

Tracking::~Tracking()
{
}
