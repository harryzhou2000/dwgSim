#pragma once

#include "dwgsimDefs.h"

#include <Eigen/Dense>

namespace DwgSim
{
    using VecX = Eigen::VectorXd;
    using Mat3X = Eigen::Matrix3Xd;
    using Vec3 = Eigen::Vector3d;
    using MatX = Eigen::MatrixXd;

    static const double verySmallDouble = 1e-200;

    inline void BSplineBases(
        int p,
        const VecX &b_knots, const VecX &b_samps,
        MatX &bases, MatX &dBases, MatX &ddBases)
    {
        int nBases = b_knots.size() - p - 1;
        if (nBases <= 0)
            throw std::runtime_error("BSpline knots number not enough");
        MatX basesC;
        MatX dBasesC, ddBasesC;
        basesC.setZero(nBases + p, b_samps.size());
        dBasesC.setZero(nBases + p, b_samps.size());
        ddBasesC.setZero(nBases + p, b_samps.size());
        bases.resize(nBases + p, b_samps.size());
        dBases.resize(nBases + p, b_samps.size());
        ddBases.resize(nBases + p, b_samps.size());

        for (int i = 0; i < basesC.rows(); i++)
        {
            if (b_knots[i] < b_knots[i + 1])
            {
                for (int j = 0; j < basesC.cols(); j++)
                    if (b_knots[i + 1] >= b_samps[j] && b_knots[i] <= b_samps[j])
                        basesC(i, j) = 1.0;
            }
        }
        basesC = basesC.array().rowwise() / (basesC.array().colwise().sum() + verySmallDouble);

        for (int jj = 1; jj <= p; jj++)
        {
            for (int i = 0; i < basesC.rows() - jj; i++)
            {
                double knotL = b_knots[i];
                double knotR = b_knots[i + jj + 1];
                double knotCL = b_knots[i + 1];
                double knotCR = b_knots[i + jj];
                double dknotLC = knotCR - knotL + verySmallDouble;
                double dknotRC = knotR - knotCL + verySmallDouble;

                bases(i, Eigen::all) =
                    (b_samps.array().transpose() - knotL) / dknotLC * basesC(i, Eigen::all).array() +
                    (knotR - b_samps.array().transpose()) / dknotRC * basesC(i + 1, Eigen::all).array();

                dBases(i, Eigen::all) =
                    (1.) / dknotLC * basesC(i, Eigen::all).array() +
                    (-1.) / dknotRC * basesC(i + 1, Eigen::all).array() +
                    (b_samps.array().transpose() - knotL) / dknotLC * dBasesC(i, Eigen::all).array() +
                    (knotR - b_samps.array().transpose()) / dknotRC * dBasesC(i + 1, Eigen::all).array();

                ddBases(i, Eigen::all) =
                    (1.) / dknotLC * dBasesC(i, Eigen::all).array() +
                    (-1.) / dknotRC * dBasesC(i + 1, Eigen::all).array() +
                    (b_samps.array().transpose() - knotL) / dknotLC * ddBasesC(i, Eigen::all).array() +
                    (knotR - b_samps.array().transpose()) / dknotRC * ddBasesC(i + 1, Eigen::all).array() +
                    (1.) / dknotLC * dBasesC(i, Eigen::all).array() +
                    (-1.) / dknotRC * dBasesC(i + 1, Eigen::all).array();
            }
            basesC = bases;
            dBasesC = dBases;
            ddBasesC = ddBases;
        }
        bases = basesC(Eigen::seq(0, nBases - 1), Eigen::all);
        dBases = dBasesC(Eigen::seq(0, nBases - 1), Eigen::all);
        ddBases = ddBasesC(Eigen::seq(0, nBases - 1), Eigen::all);
    }

    inline void CubicSplineToBSpline(
        const VecX &fit_knots,
        const Mat3X &fitPts,
        const Vec3 &startTan,
        const Vec3 &endTan,
        VecX &b_knots,
        Mat3X &b_points)
    {
        if (fit_knots.size() < 2)
            throw std::runtime_error("input vector too small");
        if (fit_knots.size() != fitPts.cols())
            throw std::runtime_error("input pts size not matching knots");
        double maxKnot = fit_knots.maxCoeff();
        double minKnot = fit_knots.minCoeff();
        double knotInterval = maxKnot - minKnot;
        if (knotInterval < verySmallDouble)
            throw numerical_error("knot input too close to same");
        for (int i = 1; i < fit_knots.size(); i++)
            if (fit_knots[i - 1] > fit_knots[i])
                throw numerical_error("knot input not ascending");

        b_knots.resize(fit_knots.size() + 6);
        b_knots(Eigen::seq(3, 3 + fit_knots.size() - 1)) = fit_knots;
        b_knots(Eigen::seq(0, 2)).setConstant(minKnot);
        b_knots(Eigen::seq(3 + fit_knots.size(), Eigen::last)).setConstant(maxKnot);

        MatX bases, dBases, ddBases;
        BSplineBases(3, b_knots, fit_knots, bases, dBases, ddBases);
        MatX A;
        A.resize(bases.rows(), bases.rows());
        A(Eigen::all, Eigen::seq(1, Eigen::last - 1)) = bases;
        if (startTan.squaredNorm() == 0)
            A(Eigen::all, 0) = ddBases(Eigen::all, 0);
        else
            A(Eigen::all, 0) = dBases(Eigen::all, 0);
        if (endTan.squaredNorm() == 0)
            A(Eigen::all, Eigen::last) = ddBases(Eigen::all, Eigen::last);
        else
            A(Eigen::all, Eigen::last) = dBases(Eigen::all, Eigen::last);
        Mat3X rhs;
        rhs.setZero(Eigen::NoChange, fit_knots.size() + 2);
        rhs(Eigen::all, Eigen::seq(1, Eigen::last - 1)) = fitPts;
        if (startTan.squaredNorm() != 0)
            rhs(Eigen::all, 0) = startTan;
        if (endTan.squaredNorm() != 0)
            rhs(Eigen::all, Eigen::last) = endTan;
        A.transposeInPlace(); // * A=A^T
        // auto decomp = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto decomp = A.fullPivLu();
        b_points = decomp.solve(rhs.transpose()).transpose();
    }

    
}