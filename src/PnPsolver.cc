/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#include <iostream>
#include "PnPsolver.h"

#include <vector>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <algorithm>

using namespace std;

namespace ORB_SLAM3
{

PnPsolver::PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches):
    pws(0), us(0), alphas(0), pcs(0),
    maximum_number_of_correspondences(0), number_of_correspondences(0),
    mnInliersi(0), mnIterations(0), mnBestInliers(0), N(0)
{
    mvpMapPointMatches = vpMapPointMatches;

    mvP2D.reserve(F.mvpMapPoints.size());
    mvSigma2.reserve(F.mvpMapPoints.size());
    mvP3Dw.reserve(F.mvpMapPoints.size());
    mvKeyPointIndices.reserve(F.mvpMapPoints.size());
    mvAllIndices.reserve(F.mvpMapPoints.size());

    int idx=0;
    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];

        if(pMP && !pMP->isBad())
        {
            const cv::KeyPoint &kp = F.mvKeysUn[i];

            mvP2D.push_back(kp.pt);
            mvSigma2.push_back(F.mvLevelSigma2[kp.octave]);

            cv::Mat Pos = pMP->GetWorldPos();
            mvP3Dw.emplace_back(Pos.at<float>(0),Pos.at<float>(1),Pos.at<float>(2));

            mvKeyPointIndices.push_back(i);
            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    fu = F.fx; fv = F.fy;
    uc = F.cx; vc = F.cy;

    SetRansacParameters();
}

PnPsolver::~PnPsolver()
{
    delete [] pws;
    delete [] us;
    delete [] alphas;
    delete [] pcs;
}

void PnPsolver::SetRansacParameters(double probability, int minInliers,
                                    int maxIterations, int minSet,
                                    float epsilon, float th2)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;
    mRansacEpsilon = epsilon;
    mRansacMinSet = minSet;

    N = mvP2D.size();
    mvbInliersi.resize(N);

    int nMinInliers = N*mRansacEpsilon;
    if(nMinInliers<mRansacMinInliers) nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet) nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;

    if(mRansacEpsilon < (float)mRansacMinInliers/N)
        mRansacEpsilon = (float)mRansacMinInliers/N;

    int nIterations;
    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mvMaxError.resize(mvSigma2.size());
    for(size_t i=0; i<mvSigma2.size(); i++)
        mvMaxError[i] = mvSigma2[i]*th2;
}

cv::Mat PnPsolver::find(vector<bool> &vbInliers, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers,nInliers);
}

cv::Mat PnPsolver::iterate(int nIterations, bool &bNoMore,
                           vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers.clear();
    nInliers=0;

    set_maximum_number_of_correspondences(mRansacMinSet);

    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts || nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;
        reset_correspondences();

        vAvailableIndices = mvAllIndices;

        for(short i = 0; i < mRansacMinSet; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            add_correspondence(
                mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,
                mvP2D[idx].x,mvP2D[idx].y);

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        compute_pose(mRi, mti);
        CheckInliers();

        if(mnInliersi>=mRansacMinInliers)
        {
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw(3,3,CV_64F,mRi);
                cv::Mat tcw(3,1,CV_64F,mti);

                Rcw.convertTo(Rcw,CV_32F);
                tcw.convertTo(tcw,CV_32F);

                mBestTcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }

            if(Refine())
            {
                nInliers = mnRefinedInliers;
                vbInliers = vector<bool>(mvpMapPointMatches.size(),false);

                for(int i=0; i<N; i++)
                    if(mvbRefinedInliers[i])
                        vbInliers[mvKeyPointIndices[i]] = true;

                return mRefinedTcw.clone();
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
    {
        bNoMore=true;

        if(mnBestInliers>=mRansacMinInliers)
        {
            nInliers=mnBestInliers;
            vbInliers = vector<bool>(mvpMapPointMatches.size(),false);

            for(int i=0; i<N; i++)
                if(mvbBestInliers[i])
                    vbInliers[mvKeyPointIndices[i]] = true;

            return mBestTcw.clone();
        }
    }

    return cv::Mat();
}

bool PnPsolver::Refine()
{
    vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());

    for(size_t i=0; i<mvbBestInliers.size(); i++)
        if(mvbBestInliers[i])
            vIndices.push_back(i);

    set_maximum_number_of_correspondences(vIndices.size());
    reset_correspondences();

    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        add_correspondence(
            mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,
            mvP2D[idx].x,mvP2D[idx].y);
    }

    compute_pose(mRi, mti);
    CheckInliers();

    mnRefinedInliers = mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    if(mnInliersi>mRansacMinInliers)
    {
        cv::Mat Rcw(3,3,CV_64F,mRi);
        cv::Mat tcw(3,1,CV_64F,mti);

        Rcw.convertTo(Rcw,CV_32F);
        tcw.convertTo(tcw,CV_32F);

        mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));

        return true;
    }
    return false;
}

void PnPsolver::CheckInliers()
{
    mnInliersi=0;

    for(int i=0; i<N; i++)
    {
        const cv::Point3f &P3Dw = mvP3Dw[i];
        const cv::Point2f &P2D = mvP2D[i];

        float Xc = mRi[0][0]*P3Dw.x + mRi[0][1]*P3Dw.y + mRi[0][2]*P3Dw.z + mti[0];
        float Yc = mRi[1][0]*P3Dw.x + mRi[1][1]*P3Dw.y + mRi[1][2]*P3Dw.z + mti[1];
        float Zc = mRi[2][0]*P3Dw.x + mRi[2][1]*P3Dw.y + mRi[2][2]*P3Dw.z + mti[2];

        float invZc = 1.0f / Zc;

        double ue = uc + fu * Xc * invZc;
        double ve = vc + fv * Yc * invZc;

        float dx = P2D.x - ue;
        float dy = P2D.y - ve;

        float error2 = dx*dx + dy*dy;

        if(error2 < mvMaxError[i])
        {
            mvbInliersi[i] = true;
            mnInliersi++;
        }
        else
            mvbInliersi[i] = false;
    }
}

void PnPsolver::choose_control_points()
{
    cws[0][0]=cws[0][1]=cws[0][2]=0;

    for(int i=0;i<number_of_correspondences;i++)
        for(int j=0;j<3;j++)
            cws[0][j]+=pws[3*i+j];

    for(int j=0;j<3;j++)
        cws[0][j]/=number_of_correspondences;

    cv::Mat PW0(number_of_correspondences,3,CV_64F);

    for(int i=0;i<number_of_correspondences;i++)
        for(int j=0;j<3;j++)
            PW0.at<double>(i,j)=pws[3*i+j]-cws[0][j];

    cv::Mat PW0tPW0, D, U;
    cv::mulTransposed(PW0, PW0tPW0, true);
    cv::SVD svd_pw0(PW0tPW0, cv::SVD::MODIFY_A);
    D = svd_pw0.w;
    U = svd_pw0.u;

    for(int i=1;i<4;i++)
    {
        double k = sqrt(D.at<double>(i-1)/number_of_correspondences);
        for(int j=0;j<3;j++)
            cws[i][j] = cws[0][j] + k*U.at<double>(j,i-1);
    }
}

void PnPsolver::compute_barycentric_coordinates()
{
    cv::Mat CC(3,3,CV_64F), CC_inv;

    for(int i=0;i<3;i++)
        for(int j=1;j<4;j++)
            CC.at<double>(i,j-1)=cws[j][i]-cws[0][i];

    cv::invert(CC, CC_inv, cv::DECOMP_SVD);

    for(int i=0;i<number_of_correspondences;i++)
    {
        double* pi = pws + 3*i;
        double* a = alphas + 4*i;

        for(int j=0;j<3;j++)
            a[1+j] =
                CC_inv.at<double>(j,0)*(pi[0]-cws[0][0]) +
                CC_inv.at<double>(j,1)*(pi[1]-cws[0][1]) +
                CC_inv.at<double>(j,2)*(pi[2]-cws[0][2]);

        a[0]=1.0-a[1]-a[2]-a[3];
    }
}

void PnPsolver::fill_M(cv::Mat &M,int row,const double* as,double u,double v)
{
    double* M1 = M.ptr<double>(row);
    double* M2 = M1 + 12;

    for(int i=0;i<4;i++)
    {
        M1[3*i]   = as[i]*fu;
        M1[3*i+1] = 0;
        M1[3*i+2] = as[i]*(uc-u);

        M2[3*i]   = 0;
        M2[3*i+1] = as[i]*fv;
        M2[3*i+2] = as[i]*(vc-v);
    }
}

double PnPsolver::compute_pose(double R[3][3], double t[3])
{
    choose_control_points();
    compute_barycentric_coordinates();

    cv::Mat M(2*number_of_correspondences,12,CV_64F);

    for(int i=0;i<number_of_correspondences;i++)
        fill_M(M,2*i,alphas+4*i,us[2*i],us[2*i+1]);

    cv::Mat MtM,D,U;
    cv::mulTransposed(M,MtM,true);
    cv::SVD svd_mtm(MtM, cv::SVD::MODIFY_A);
    D = svd_mtm.w;
    U = svd_mtm.u;

    cv::Mat L_6x10(6,10,CV_64F);
    cv::Mat Rho(6,1,CV_64F);

    compute_L_6x10(U.ptr<double>(), L_6x10.ptr<double>());
    compute_rho(Rho.ptr<double>());

    double Betas[4][4], errors[4];
    double Rs[4][3][3], ts[4][3];

    find_betas_approx_1(L_6x10,Rho,Betas[1]);
    gauss_newton(L_6x10,Rho,Betas[1]);
    errors[1]=compute_R_and_t(U.ptr<double>(),Betas[1],Rs[1],ts[1]);

    find_betas_approx_2(L_6x10,Rho,Betas[2]);
    gauss_newton(L_6x10,Rho,Betas[2]);
    errors[2]=compute_R_and_t(U.ptr<double>(),Betas[2],Rs[2],ts[2]);

    find_betas_approx_3(L_6x10,Rho,Betas[3]);
    gauss_newton(L_6x10,Rho,Betas[3]);
    errors[3]=compute_R_and_t(U.ptr<double>(),Betas[3],Rs[3],ts[3]);

    int best=1;
    if(errors[2]<errors[best]) best=2;
    if(errors[3]<errors[best]) best=3;

    copy_R_and_t(Rs[best],ts[best],R,t);

    return errors[best];
}

void PnPsolver::find_betas_approx_1(const cv::Mat &L_6x10,const cv::Mat &Rho,double *betas)
{
    double l_6x4[24], b4[4];

    cv::Mat L_6x4(6,4,CV_64F,l_6x4);
    cv::Mat B4(4,1,CV_64F,b4);

    for(int i=0;i<6;i++)
    {
        L_6x4.at<double>(i,0)=L_6x10.at<double>(i,0);
        L_6x4.at<double>(i,1)=L_6x10.at<double>(i,1);
        L_6x4.at<double>(i,2)=L_6x10.at<double>(i,3);
        L_6x4.at<double>(i,3)=L_6x10.at<double>(i,6);
    }

    cv::solve(L_6x4,Rho,B4,cv::DECOMP_SVD);

    betas[0]=sqrt(fabs(b4[0]));
    betas[1]=b4[1]/betas[0];
    betas[2]=b4[2]/betas[0];
    betas[3]=b4[3]/betas[0];
}

}
