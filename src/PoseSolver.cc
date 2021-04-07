/*
 * @Author: your name
 * @Date: 2021-03-28 17:41:34
 * @LastEditTime: 2021-04-07 10:21:54
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /极线可视化/src/PoseSolver.cc
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <cassert>

// #include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Config.h"
#include "PoseSolver.h"
#include "Optimization.h"

#define DEBUG
#ifdef DEBUG
    #define CHECK_INFO(x) std::cout << "[DEBUG] " << x << std::endl;
    #define CHECK_INFO_2(x,y) std::cout << "[DEBUG] " << x << y << std::endl;
#else
    #define CHECK_INFO(x) /\
    /std::cout << x << std::endl;
    #define CHECK_INFO_2(x,y) //std::cout << "[DEBUG] " << x << y << std::endl;
#endif

PoseSolver::PoseSolver(std::vector<cv::Point3f> &coord_0, std::vector<cv::Point3f> &coord_1, Config &config, int initFlag)
    : PointsInPixelVec_0(coord_0),PointsInPixelVec_1(coord_1), setting(config)
{
    PixelPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(0, &PointsInPixelVec_0));
    PixelPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(1, &PointsInPixelVec_1));
    WorldPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(0, &PointsInWorldVec_0));
    WorldPointsDict.insert(std::pair< int, std::vector<cv::Point3f>* >(1, &PointsInWorldVec_1));
    SetOrigin(0);
}

PoseSolver::PoseSolver(std::vector<cv::Point3f> &ptInWorld_0, std::vector<cv::Point3f> &ptInPixel_1, Config &config) : PointsInPixelVec_0(ptInWorld_0),PointsInPixelVec_1(ptInPixel_1), setting(config)
{
    SetOrigin(1);
}


PoseSolver::~PoseSolver()
{
}

void PoseSolver::SetOrigin(const int &flag){
    cv::Mat cvK = setting.ip.K;
    // cv::cv2eigen(cvK,eigenK);
    eigenK <<   cvK.at<float>(0,0), 0 , cvK.at<float>(0,2), 0,
                cvK.at<float>(1,1), cvK.at<float>(1,2),
                0, 0, 1;
    if (flag==0){
        Pose_0 = Eigen::Matrix4f::Identity();
        CamOrientation_0 << 0, 0, 1;
        Unproject(0,Pose_0);
    }else{
        Pose_0 = Eigen::Matrix4f::Identity();
        CamOrientation_0 << 0, 0, 1;
    }

}

void PoseSolver::Unproject(const int &PoseId, const Eigen::Matrix4f &T)
{
    for (size_t i(0); i< PixelPointsDict[PoseId]->size(); ++i){
        float u = PixelPointsDict[PoseId]->at(i).x;
        float v = PixelPointsDict[PoseId]->at(i).y;
        float z = PixelPointsDict[PoseId]->at(i).z;
        if (z <= 0) continue;
        Eigen::Vector3f PixelPoint(u, v, 1);
        Eigen::Vector3f PointInCam = z*(eigenK.inverse()*PixelPoint);
        // std::cout << "R is " << std::endl << T.block(0,0,3,3).transpose() << std::endl;
        // std::cout << "t is " << std::endl << -T.block(0,0,3,3).transpose()*T.block<3,1>(0,3) << std::endl;
        Eigen::Vector3f PointInWorld = T.block(0,0,3,3).transpose()*PointInCam
            -T.block(0,0,3,3).transpose()*T.block<3,1>(0,3);
        WorldPointsDict[PoseId]->emplace_back(PointInWorld(0), PointInWorld(1), PointInWorld(2));
    }
    // // CHECK POINT
    // std::cout << "CHECK POINT " << std::endl;
    // for (size_t i=0; i<WorldPointsDict[PoseId]->size(); ++i){
    //     std::cout << WorldPointsDict[PoseId]->at(i).x << "\t\t" <<
    //         WorldPointsDict[PoseId]->at(i).y << "\t\t" <<
    //         WorldPointsDict[PoseId]->at(i).z << std::endl;
    // }
}

void PoseSolver::ComputePnP()
{
    // CheckReProjrctError();
    // std::vector<int> ls={10,20,30,40};
    // KneipPnP(ls,PointsInWorldVec_0,PointsInPixelVec_1);
    Optimization opts(setting);
    std::vector<int> Interior;
    Interior.resize(PointsInWorldVec_0.size());
    opts.Kneip_Ransac(PointsInWorldVec_0, PointsInPixelVec_1, Interior, T12_ransac);


    // CHECK POINT BEGIN
    CHECK_INFO("Before: ");
    CHECK_INFO_2("Size: ",PointsInWorldVec_0.size());
    double error = Average_ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, T12_ransac);
    CHECK_INFO_2("average error: ",error);

    std::vector< cv::Point3f > tmp_PW0, tmp_PP1;
    for (size_t i(0); i<Interior.size(); ++i){
        if(Interior[i]==1){
            tmp_PW0.emplace_back(PointsInWorldVec_0[i]);
            tmp_PP1.emplace_back(PointsInPixelVec_1[i]);
        }
    }

    CHECK_INFO("After: ");
    PointsInWorldVec_0 = tmp_PW0;
    PointsInPixelVec_1 = tmp_PP1;
    CHECK_INFO_2("Size: ",PointsInWorldVec_0.size());
    double Interior_Error = Average_ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, T12_ransac);
    CHECK_INFO_2("average error: ",Interior_Error);

    // CHECK POINT END

}


void PoseSolver::KneipPnP(std::vector<int> &idVec
    ,std::vector<cv::Point3f> &PointsInWorldVec_0
    ,std::vector<cv::Point3f> &PointsInPixelVec_1)
{
    const int a = idVec[0];
    const int b = idVec[1];
    const int c = idVec[2];
    const int d = idVec[3];
    Eigen::Vector3f f1, f2, f3, p1, p2, p3;
    Eigen::Vector3f tx, ty, tz;
    p1 << PointsInPixelVec_1[a].x, PointsInPixelVec_1[a].y, 1;
    p2 << PointsInPixelVec_1[b].x, PointsInPixelVec_1[b].y, 1;
    p3 << PointsInPixelVec_1[c].x, PointsInPixelVec_1[c].y, 1;

    double const colinear_threshold = 1e-10;
    if (((p2 - p1).cross(p3 - p1)).squaredNorm() < colinear_threshold)
        return;

    f1 << eigenK.inverse() * p1;
    f2 << eigenK.inverse() * p2;
    f3 << eigenK.inverse() * p3;
    f1 = f1.normalized();
    f2 = f2.normalized();
    f3 = f3.normalized();

    tx = f1;
    tz = (f1.cross(f2)).normalized();
    ty = tz.cross(tx);

    Eigen::Matrix3f T;
    T.row(0) = tx.transpose();
    T.row(1) = ty.transpose();
    T.row(2) = tz.transpose();
    f3 = T * f3;

    if ( f3(2)> 0.0){
        std::swap(f1, f2);
        std::swap(p1, p2);

        tx = f1;
        tz = (f1.cross(f2)).normalized();
        ty = tz.cross(tx);
        T.row(0) = tx.transpose();
        T.row(1) = ty.transpose();
        T.row(2) = tz.transpose();
        f3 = T * f3;
    }
    Eigen::Vector3f nx, ny, nz, P1, P2, P3;

    P1 << PointsInWorldVec_0[a].x, PointsInWorldVec_0[a].y, PointsInWorldVec_0[a].z;
    P2 << PointsInWorldVec_0[b].x, PointsInWorldVec_0[b].y, PointsInWorldVec_0[b].z;
    P3 << PointsInWorldVec_0[c].x, PointsInWorldVec_0[c].y, PointsInWorldVec_0[c].z;

    Eigen::Vector3f P12(P2-P1);
    Eigen::Vector3f P13(P3-P1);

    nx = P12.normalized();
    nz = nx.cross(P13).normalized();
    ny = nz.cross(nx);

    Eigen::Matrix3f N;
    N.row(0) = nx.transpose();
    N.row(1) = ny.transpose();
    N.row(2) = nz.transpose();
    P3 = N*(P13);

    double d12 = P12.norm();
    double cos_beta = f1.dot(f2);
    double bais = sqrt(1.0/( 1.0 - pow(cos_beta,2) )-1);

    if (cos_beta < 0.0)
        bais = -bais;

    double pha_1 = f3(0)/f3(2);
    double pha_2 = f3(1)/f3(2);

    Eigen::Matrix<float,5,1> coff;
    std::vector<double> real_roots;

    coff(4) =-2.0*pha_1*pha_2*P3(0)*pow(P3(1),2)*d12*bais
            + pow(pha_2*P3(1)*d12,2)
            + 2.0*pow(P3(0),3)*d12 - pow(P3(0)*d12,2)
            + pow(pha_2*P3(0)*P3(1),2) -pow(P3(0),4)
            - 2.0*pow(pha_2*P3(1),2)*P3(0)*d12
            + pow(pha_1*P3(0)*P3(1),2)
            + pow(pha_2*P3(1)*bais*d12,2);

    coff(3) = 2.0*pow(P3(0),2)*P3(1)*d12*bais
            + 2.0*pha_1*pha_2*d12*pow(P3(1),3)
            - 2.0*pow(pha_2,2)*pow(P3(1),3)*d12*bais
            - 2.0*P3(0)*P3(1)*bais*pow(d12,2);

    coff(2) = - pow(pha_2*P3(0)*P3(1),2)
            - pow(pha_2*P3(1)*d12*bais,2)
            - pow(pha_2*P3(1)*d12,2)
            + pow(P3(1),4)*(pow(pha_1,2)+pow(pha_2,2))
            + 2.0*P3(0)*d12*pow(P3(1),2)
            + 2.0*pha_1*pha_2*P3(0)*d12*bais*pow(P3(1),2)
            - pow(pha_1*P3(0)*P3(1),2)
            + 2.0*pow(pha_2*P3(1),2)*P3(0)*d12
            - pow(P3(1)*d12*bais,2) -2.0*pow(P3(0)*P3(1),2);

    coff(1) = 2.0 *pow(P3(1),3)*d12*bais
            + 2.0*pow(pha_2,2)*pow(P3(1),3)*d12*bais
            - 2.0*pha_1*pha_2*d12*pow(P3(1),3);

    coff(0) = -(pow(pha_2,2) + pow(pha_1,2) + 1)*pow(P3(1),4);

    solve_quartic_roots(coff,real_roots);
    assert(real_roots.size()==4);

    double minError = 9999;
    for (size_t i=0; i<4; ++i){
        double cot_alpha = (pha_1/pha_2*P3(0)+real_roots[i]*P3(1)-d12*bais)
            /(pha_1/pha_2*real_roots[i]*P3(1)- P3(0)+d12);

        // \theta \in [0,\pi];
        double cos_theta = real_roots[i];
        double sin_theta = sqrt(1-pow(cos_theta,2));
        // \alpha \in [0,\pi];
        double sin_alpha = sqrt(1.0/(pow(cot_alpha,2)+1));
        double cos_alpha = sqrt(1.0-(pow(sin_alpha,2)));
        if (cot_alpha < 0.0)
            cos_alpha = -cos_alpha;

        Eigen::Matrix3f R,Q;
        Eigen::Vector3f C_yita;
        C_yita(0) = d12*cos_alpha*(sin_alpha*bais+cos_alpha);
        C_yita(1) = d12*sin_alpha*cos_theta*(sin_alpha*bais+cos_alpha);
        C_yita(2) = d12*sin_alpha*sin_theta*(sin_alpha*bais+cos_alpha);

        Q << -cos_alpha, -sin_alpha*cos_theta, -sin_alpha*sin_theta,
             sin_alpha,  -cos_alpha*cos_theta, -cos_alpha*sin_theta,
             0.0,          -sin_theta,           cos_theta;

        CamOrientation_1 = P1 + N.transpose()*C_yita;
        R = N.transpose()*(Q.transpose()*T);
        R.transposeInPlace();
        Eigen::Vector3f t = -R*CamOrientation_1;

        Eigen::Matrix4f T12_;
        T12_ << R(0,0), R(0,1), R(0,2), t(0),
                R(1,0), R(1,1), R(1,2), t(1),
                R(2,0), R(2,1), R(2,2), t(2),
                0     , 0     , 0     , 1   ;
        double error = ReProjectError(PointsInWorldVec_0,PointsInPixelVec_1, d, T12_);
        if (minError > error){
            minError = error;
            T12 = T12_;
        }

    }
    // CHECK POINT
    // std::cout << "[TEST] MIN ERROR IS: " << minError << std::endl;
}


double PoseSolver::Average_ReProjectError(std::vector<cv::Point3f>&PointsInWorldVec_0
    ,std::vector<cv::Point3f>&PointsInPixelVec_1, Eigen::Matrix4f &T)
{
    assert(PointsInWorldVec_0.size() == PointsInPixelVec_1.size() );
    assert(!PointsInWorldVec_0.empty());
    double total_error = 0;
    for (size_t i(0); i<PointsInWorldVec_0.size();++i){
        double error = ReProjectError(PointsInWorldVec_0, PointsInPixelVec_1, i, T);
        total_error += error;
        // CHECK_INFO_2("error : ", error);
    }

    return total_error/PointsInWorldVec_0.size();
}


double PoseSolver::ReProjectError(std::vector<cv::Point3f>&PointsInWorldVec_0
    ,std::vector<cv::Point3f>&PointsInPixelVec_1,const int &i, Eigen::Matrix4f &T)
{
    Eigen::Vector3f u4, u4_;
    u4 << PointsInPixelVec_1[i].x, PointsInPixelVec_1[i].y, 1;
    Eigen::Vector4f P4;
    P4 << PointsInWorldVec_0[i].x, PointsInWorldVec_0[i].y, PointsInWorldVec_0[i].z, 1;
    Eigen::Vector4f CamCoord = T * P4;
    Project(CamCoord,u4_);
    double error = sqrt(pow(u4(0)-u4_(0),2)+pow(u4(1)-u4_(1),2));
    // std::cout << "[TEST] ERROR IS :" << error << std::endl;
    return error;
}

void PoseSolver::Project(Eigen::Vector4f &CamCoord, Eigen::Vector3f & PixelCoord){
    PixelCoord(0) = setting.ip.fx * CamCoord(0)/CamCoord(2) + setting.ip.cx;
    PixelCoord(1) = setting.ip.fy * CamCoord(1)/CamCoord(2) + setting.ip.cy;
    PixelCoord(2) = 1;
}


void PoseSolver::solve_quartic_roots(Eigen::Matrix<float,5,1> const& factors, std::vector<double> &real_roots)
{
    double const A = factors(0);
    double const B = factors(1);
    double const C = factors(2);
    double const D = factors(3);
    double const E = factors(4);

    double const A2 = A * A;
    double const B2 = B * B;
    double const A3 = A2 * A;
    double const B3 = B2 * B;
    double const A4 = A3 * A;
    double const B4 = B3 * B;

    double const alpha = -3.0 * B2 / (8.0 * A2) + C / A;
    double const beta = B3 / (8.0 * A3)- B * C / (2.0 * A2) + D / A;
    double const gamma = -3.0 * B4 / (256.0 * A4) + B2 * C / (16.0 * A3) - B * D / (4.0 * A2) + E / A;

    double const alpha2 = alpha * alpha;
    double const alpha3 = alpha2 * alpha;
    double const beta2 = beta * beta;

    std::complex<double> P(-alpha2 / 12.0 - gamma, 0.0);
    std::complex<double> Q(-alpha3 / 108.0 + alpha * gamma / 3.0 - beta2 / 8.0, 0.0);
    std::complex<double> R = -Q / 2.0 + std::sqrt(Q * Q / 4.0 + P * P * P / 27.0);

    std::complex<double> U = std::pow(R, 1.0 / 3.0);
    std::complex<double> y = (U.real() == 0.0)
                             ? -5.0 * alpha / 6.0 - std::pow(Q, 1.0 / 3.0)
                             : -5.0 * alpha / 6.0 - P / (3.0 * U) + U;

    std::complex<double> w = std::sqrt(alpha + 2.0 * y);
    std::complex<double> part1 = -B / (4.0 * A);
    std::complex<double> part2 = 3.0 * alpha + 2.0 * y;
    std::complex<double> part3 = 2.0 * beta / w;

    std::complex<double> complex_roots[4];
    complex_roots[0] = part1 + 0.5 * (w + std::sqrt(-(part2 + part3)));
    complex_roots[1] = part1 + 0.5 * (w - std::sqrt(-(part2 + part3)));
    complex_roots[2] = part1 + 0.5 * (-w + std::sqrt(-(part2 - part3)));
    complex_roots[3] = part1 + 0.5 * (-w - std::sqrt(-(part2 - part3)));

    for (int i = 0; i < 4; ++i)
        real_roots.push_back(complex_roots[i].real());
}

void PoseSolver::CheckReProjrctError()
{
    // 使用检查机制会破坏原有的数据，请仅在检查时才能使用
    cv::Point3f p1(-2.57094,-0.217018, 6.05338);
    cv::Point3f p2(-0.803123, 0.251818, 6.98383);
    cv::Point3f p3(2.05584, -0.607918, 7.52573);
    cv::Point3f p4(-0.62611418962478638, -0.80525958538055419, 6.7783102989196777);
    PointsInWorldVec_0[10] = p1;
    PointsInWorldVec_0[20] = p2;
    PointsInWorldVec_0[30] = p3;
    PointsInWorldVec_0[40] = p4;
    cv::Point3f uv1(-0.441758,-0.185523,1);
    cv::Point3f uv2(-0.135753,-0.0920593,1);
    cv::Point3f uv3(0.243795,-0.192743,1);
    cv::Point3f uv4(-0.11282696574926376,-0.24667978286743164,1);
    PointsInPixelVec_1[10] = uv1;
    PointsInPixelVec_1[20] = uv2;
    PointsInPixelVec_1[30] = uv3;
    PointsInPixelVec_1[40] = uv4;
    setting.ip.fx = 0.972222;
    setting.ip.fy = 0.972222;
    setting.ip.cx = 0.0;
    setting.ip.cy = 0.0;
    eigenK << setting.ip.fx, 0, setting.ip.cx, 0, setting.ip.fy, setting.ip.cy, 0, 0, 1;
    std::vector<int> ls={10,20,30,40};
    KneipPnP(ls,PointsInWorldVec_0,PointsInPixelVec_1);
    double error = ReProjectError(PointsInWorldVec_0,PointsInPixelVec_1, 40, T12);
    std::cout << "[INFO] ERROR IS: " << error << std::endl;

    if (error > 0.01)
        std::cerr << "[ERRO] PNP ERROR!" << std::endl;
    else
        std::cerr << "[INFO] PNP OK." << std::endl;

    exit(0);

}
