/*
 * @Author: your name
 * @Date: 2021-04-02 10:37:48
 * @LastEditTime: 2021-04-07 09:59:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /SLAM-Demo/src/Optimization.cc
 */

#include <cmath>
#include <iostream>
#include <cassert>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdlib> // 随机数

#include "Config.h"
#include "Optimization.h"
#include "PoseSolver.h"

#define DEBUG
#ifdef DEBUG
    #define CHECK_INFO(x) std::cout << "[DEBUG] " << x << std::endl;
    #define CHECK_INFO_2(x,y) std::cout << "[DEBUG] " << x << y << std::endl;
#else
    #define CHECK_INFO(x) /\
    /std::cout << x << std::endl;
    #define CHECK_INFO_2(x,y) //std::cout << "[DEBUG] " << x << y << std::endl;
#endif

Optimization::Optimization(Config &config): setting(config)
{
    std::cout << "[INFO] Optimization Start." << std::endl;
}

Optimization::~Optimization()
{
}

// template<T, N>
// void Optimization::LeastSquare(std::vector<T> &Observation, N &Result)
// {
// }

void Optimization::Kneip_Ransac(std::vector<cv::Point3f> &PointsInWorldVec_0
    , std::vector<cv::Point3f> &PointsInPixelVec_1
    , std::vector<int> &Interior
    , Eigen::Matrix4f &Pose )
{
    assert(PointsInWorldVec_0.size() == PointsInPixelVec_1.size());

    unsigned seed = time(0);
    srand(seed);

    std::vector<int> InlierNumVec;
    std::vector< std::vector<int> > InlierIDVec;
    std::vector< Eigen::Matrix4f > PoseVec;

    PoseSolver ps(PointsInWorldVec_0, PointsInPixelVec_1, setting);

    for (size_t i(0); i<setting.oc.maxRansacIter; ++i){
        std::set<int> idSet;
        std::vector<int> idVec;
        while( idSet.size() <4 ){
            int rand_num = static_cast<int>(rand()%(PointsInWorldVec_0.size()));
            idSet.insert(rand_num);
        }

        for (std::set<int>::iterator iter = idSet.begin(); iter!=idSet.end(); ++iter){
            idVec.push_back(*iter);
            // CHECK_INFO_2("rand_num is: ", idVec.back());
        }
        assert(idVec.size()==4);

        ps.KneipPnP(idVec, PointsInWorldVec_0, PointsInPixelVec_1);
        Eigen::Matrix4f T12 = ps.GetT12();
        PoseVec.emplace_back(T12);

        std::vector<int> tmp;
        int cnt = 0;
        for(size_t j(0); j<PointsInPixelVec_1.size();++j)
        {
            double error = ps.ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, j, T12);
            // CHECK_INFO_2("error : ", error);
            if(error < setting.oc.ErrorTH){
                cnt++;
                tmp.push_back(j);
            }
        }
        InlierIDVec.emplace_back(tmp);
        InlierNumVec.emplace_back(cnt);
    }

    std::vector<int>::iterator maxInfo = max_element(InlierNumVec.begin(),InlierNumVec.end());
    int max_pos = std::distance(InlierNumVec.begin(), maxInfo);
    Pose = PoseVec[max_pos];
    for(size_t k(0); k<InlierIDVec[max_pos].size();++k){
        int id = InlierIDVec[max_pos][k];
        Interior[id] = 1;
    }

    CHECK_INFO_2("max id: ", max_pos);

}



