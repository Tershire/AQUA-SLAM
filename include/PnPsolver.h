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

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core.hpp>
#include <vector>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM3
{

class PnPsolver
{
public:
    PnPsolver(const Frame &F, const std::vector<MapPoint*> &vpMapPointMatches);
    ~PnPsolver();

    void SetRansacParameters(double probability = 0.99, int minInliers = 8 ,
                             int maxIterations = 300, int minSet = 4,
                             float epsilon = 0.4, float th2 = 5.991);

    cv::Mat find(std::vector<bool> &vbInliers, int &nInliers);
    cv::Mat iterate(int nIterations, bool &bNoMore,
                    std::vector<bool> &vbInliers, int &nInliers);

private:

    // ===== RANSAC =====
    double mRansacProb;
    int mRansacMinInliers;
    int mRansacMaxIts;
    float mRansacEpsilon;
    int mRansacMinSet;

    int N;

    // ===== Data =====
    std::vector<MapPoint*> mvpMapPointMatches;
    std::vector<cv::Point2f> mvP2D;
    std::vector<cv::Point3f> mvP3Dw;
    std::vector<float> mvSigma2;
    std::vector<float> mvMaxError;

    std::vector<int> mvKeyPointIndices;
    std::vector<size_t> mvAllIndices;

    // ===== Inliers =====
    std::vector<bool> mvbInliersi;
    std::vector<bool> mvbBestInliers;
    std::vector<bool> mvbRefinedInliers;

    int mnInliersi;
    int mnBestInliers;
    int mnRefinedInliers;
    int mnIterations;

    // ===== Pose =====
    double mRi[3][3];
    double mti[3];

    cv::Mat mBestTcw;
    cv::Mat mRefinedTcw;

    // ===== Camera intrinsics =====
    float fu, fv, uc, vc;

    // ===== EPnP internal =====
    double *pws;
    double *us;
    double *alphas;
    double *pcs;

    int maximum_number_of_correspondences;
    int number_of_correspondences;

    double cws[4][3];

    // ===== Core Functions =====
    bool Refine();
    void CheckInliers();

    void choose_control_points();
    void compute_barycentric_coordinates();

    void fill_M(cv::Mat &M, int row, const double *alphas,
                const double u, const double v);

    double compute_pose(double R[3][3], double t[3]);

    void find_betas_approx_1(const cv::Mat &L_6x10,
                             const cv::Mat &Rho,
                             double *betas);

    // ===== Missing but REQUIRED =====
    void find_betas_approx_2(const cv::Mat &L_6x10,
                             const cv::Mat &Rho,
                             double *betas);

    void find_betas_approx_3(const cv::Mat &L_6x10,
                             const cv::Mat &Rho,
                             double *betas);

    void gauss_newton(const cv::Mat &L_6x10,
                      const cv::Mat &Rho,
                      double betas[4]);

    void compute_L_6x10(const double *ut, double *l_6x10);

    void compute_rho(double *rho);

    double compute_R_and_t(const double *ut,
                           const double *betas,
                           double R[3][3],
                           double t[3]);

    void copy_R_and_t(const double R_src[3][3],
                      const double t_src[3],
                      double R_dst[3][3],
                      double t_dst[3]);

    void set_maximum_number_of_correspondences(int n);

    void reset_correspondences();

    void add_correspondence(double X, double Y, double Z,
                            double u, double v);
};

}

#endif
