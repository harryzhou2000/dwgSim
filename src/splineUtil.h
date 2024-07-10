#pragma once

#include "dwgsimDefs.h"

#include <Eigen/Dense>

namespace DwgSim
{
    using VecX = Eigen::VectorXd;
    using Mat3X = Eigen::Matrix3Xd;
    using Vec3 = Eigen::Vector3d;

    void CubicSplineToBSpline(
        const VecX &fit_knots,
        const Mat3X &fitPts,
        const Vec3 &startTan,
        const Vec3 &endTan,
        VecX &b_knots,
        Mat3X &b_points)
    {
    }
}