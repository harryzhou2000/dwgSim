#pragma once

#include "dwgsimDefs.h"

#include <Eigen/Dense>
#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/allocators.h>

namespace DwgSim
{
    using VecX = Eigen::VectorXd;
    using Mat3X = Eigen::Matrix3Xd;
    using Vec3 = Eigen::Vector3d;
    using MatX = Eigen::MatrixXd;

    static const double verySmallDouble = 1e-200;

    inline VecX RapidJsonGetVecX(const rapidjson::Value &a)
    {
        if (!a.IsArray())
            throw std::runtime_error("input is not array");
        VecX ret;
        ret.resize(a.Size());
        for (int64_t i = 0; i < ret.size(); i++)
            ret[i] = a[i].GetDouble();
        return ret;
    }

    inline VecX RapidJsonGetVecX4th(const rapidjson::Value &a)
    {
        if (!a.IsArray())
            throw std::runtime_error("input is not array");
        VecX ret;
        ret.resize(a.Size());
        for (int64_t i = 0; i < ret.size(); i++)
            ret[i] = a[i][3].GetDouble();
        return ret;
    }

    inline Vec3 RapidJsonGetVec3(const rapidjson::Value &a)
    {
        if (!a.IsArray() || a.Size() < 3)
            throw std::runtime_error("input is not at least 3-sized array");
        Vec3 ret;
        for (int64_t i = 0; i < 3; i++)
            ret[i] = a[i].GetDouble();
        return ret;
    }

    inline Mat3X RapidJsonGetMat3X(const rapidjson::Value &a)
    {
        if (!a.IsArray())
            throw std::runtime_error("input is not array");
        Mat3X ret;
        ret.resize(Eigen::NoChange, a.Size());
        for (int64_t i = 0; i < a.Size(); i++)
            ret(Eigen::all, i) = RapidJsonGetVec3(a[i]);
        return ret;
    }

    inline rapidjson::Value VecXGetRapidJson(
        const VecX &m,
        rapidjson::MemoryPoolAllocator<> &alloc)
    {
        rapidjson::Value ret(rapidjson::kArrayType);
        ret.Reserve(m.size(), alloc);
        for (int64_t i = 0; i < m.size(); i++)
            ret.PushBack(m[i], alloc);
        return ret;
    }

    inline rapidjson::Value Vec3GetRapidJson(
        const Vec3 &m,
        rapidjson::MemoryPoolAllocator<> &alloc)
    {
        rapidjson::Value ret(rapidjson::kArrayType);
        ret.Reserve(3, alloc);
        for (int i = 0; i < 3; i++)
            ret.PushBack(m[i], alloc);
        return ret;
    }

    inline rapidjson::Value Mat3XGetRapidJson(
        const Mat3X &m,
        rapidjson::MemoryPoolAllocator<> &alloc)
    {
        rapidjson::Value ret(rapidjson::kArrayType);
        ret.Reserve(m.cols(), alloc);
        for (int64_t i = 0; i < m.cols(); i++)
            ret.PushBack(Vec3GetRapidJson(m(Eigen::all, i), alloc), alloc);
        return ret;
    }

    inline void BSplineBases(
        int p,
        const VecX &b_knots, const VecX &b_samps,
        MatX &bases, MatX &dBases, MatX &ddBases)
    {
        int nBases = b_knots.size() - p - 1;
        if (nBases <= 0)
            throw std::runtime_error("BSpline knots number not enough");
        MatX basesC, dBasesC, ddBasesC;
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

    inline void BSplineBasesPeriodic(
        int p,
        const VecX &b_knots, const VecX &b_samps,
        MatX &bases, MatX &dBases, MatX &ddBases)
    {
        int nBases = b_knots.size() - 1;
        if (nBases <= 0)
            throw std::runtime_error("BSpline knots number not enough");
        MatX basesC, dBasesC, ddBasesC;
        basesC.setZero(nBases, b_samps.size());
        dBasesC.setZero(nBases, b_samps.size());
        ddBasesC.setZero(nBases, b_samps.size());
        bases.resize(nBases, b_samps.size());
        dBases.resize(nBases, b_samps.size());
        ddBases.resize(nBases, b_samps.size());
        double period = b_knots.maxCoeff() - b_knots.minCoeff();
        double t0 = b_knots.minCoeff();
        VecX b_knotsP;
        b_knotsP.resize(nBases + p + 1);
        for (int i = 0; i < b_knotsP.size(); i++)
            b_knotsP[i] = b_knots[i % nBases] + (i / nBases) * period;

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
        // std::cout << "===Bases:\n";
        // std::cout << basesC << std::endl;
        for (int jj = 1; jj <= p; jj++)
        {
            for (int i = 0; i < basesC.rows(); i++)
            {
                double knotL = b_knotsP[i];
                double knotR = b_knotsP[(i + jj + 1)];
                double knotCL = b_knotsP[(i + 1)];
                double knotCR = b_knotsP[(i + jj)];
                // std::cout << knotL << " " << knotCL << " " << knotCR << " " << knotR << std::endl;
                double dknotLC = knotCR - knotL + verySmallDouble;
                double dknotRC = knotR - knotCL + verySmallDouble;
                double cKnotCLR = 0.5 * (knotCL + knotCR);

                VecX b_sampsC = b_samps;
                int maxIt = (p + 1) / nBases + 2;
                for (auto &v : b_sampsC)
                    for (int iter = 0; iter < maxIt; iter++)
                    {
                        // the periodic coord value closest to [knotLC knotRC] is valid
                        if (std::abs(v + period - cKnotCLR) > std::abs(v - cKnotCLR))
                            break;
                        else
                            v += period;
                    }

                //? need shifting?
                bases((i + 0) % nBases, Eigen::all) =
                    (b_sampsC.array().transpose() - knotL) / dknotLC * basesC(i, Eigen::all).array() +
                    (knotR - b_sampsC.array().transpose()) / dknotRC * basesC((i + 1) % nBases, Eigen::all).array();

                dBases((i + 0) % nBases, Eigen::all) =
                    (1.) / dknotLC * basesC(i, Eigen::all).array() +
                    (-1.) / dknotRC * basesC((i + 1) % nBases, Eigen::all).array() +
                    (b_sampsC.array().transpose() - knotL) / dknotLC * dBasesC(i, Eigen::all).array() +
                    (knotR - b_sampsC.array().transpose()) / dknotRC * dBasesC((i + 1) % nBases, Eigen::all).array();

                ddBases((i + 0) % nBases, Eigen::all) =
                    (1.) / dknotLC * dBasesC(i, Eigen::all).array() +
                    (-1.) / dknotRC * dBasesC((i + 1) % nBases, Eigen::all).array() +
                    (b_sampsC.array().transpose() - knotL) / dknotLC * ddBasesC(i, Eigen::all).array() +
                    (knotR - b_sampsC.array().transpose()) / dknotRC * ddBasesC((i + 1) % nBases, Eigen::all).array() +
                    (1.) / dknotLC * dBasesC(i, Eigen::all).array() +
                    (-1.) / dknotRC * dBasesC((i + 1) % nBases, Eigen::all).array();
            }
            // std::cout << "===Bases:\n";
            // std::cout << bases << std::endl;
            basesC = bases;
            dBasesC = dBases;
            ddBasesC = ddBases;
        }
        bases = basesC(Eigen::seq(0, nBases - 1), Eigen::all);
        dBases = dBasesC(Eigen::seq(0, nBases - 1), Eigen::all);
        ddBases = ddBasesC(Eigen::seq(0, nBases - 1), Eigen::all);
    }

    inline void PeriodicSplineToOpenSpline(
        const VecX &b_knots_p, const Mat3X &b_points_p,
        VecX &b_knots, Mat3X &b_points)
    {
        // Note: this only fits to cubic b-spline, using standard
        // double maxKnot = b_knots_p.maxCoeff();
        // double minKnot = b_knots_p.minCoeff();
        // double knotInterval = maxKnot - minKnot;
        // if (knotInterval < verySmallDouble)
        //     throw numerical_error("knot input too close to same");
        // for (int i = 1; i < b_knots_p.size(); i++)
        //     if (b_knots_p[i - 1] > b_knots_p[i])
        //         throw numerical_error("knot input not ascending");

        // b_knots.resize(b_knots_p.size() + 6);
        // b_knots(Eigen::seq(3, 3 + b_knots_p.size() - 1)) = b_knots_p;
        // b_knots(Eigen::seq(0, 2)).setConstant(minKnot);
        // b_knots(Eigen::seq(3 + b_knots_p.size(), Eigen::last)).setConstant(maxKnot);

        // MatX bases, dBases, ddBases;
        // BSplineBases(3, b_knots, b_knots_p, bases, dBases, ddBases);
    }

    inline void CubicSplineToBSpline(
        const VecX &fit_knots,
        const Mat3X &fitPts,
        const Vec3 &startTan,
        const Vec3 &endTan,
        VecX &b_knots,
        Mat3X &b_points,
        bool periodic = false)
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
        Mat3X rhs;
        rhs.setZero(Eigen::NoChange, fit_knots.size() + 2);
        rhs(Eigen::all, Eigen::seq(1, Eigen::last - 1)) = fitPts;
        if (periodic)
        {
            A(Eigen::all, 0) = ddBases(Eigen::all, 0) - ddBases(Eigen::all, Eigen::last);
            A(Eigen::all, Eigen::last) = dBases(Eigen::all, 0) - dBases(Eigen::all, Eigen::last);
        }
        else
        {
            if (startTan.squaredNorm() == 0)
                A(Eigen::all, 0) = ddBases(Eigen::all, 0);
            else
                A(Eigen::all, 0) = dBases(Eigen::all, 0),
                              rhs(Eigen::all, 0) = startTan; //* the startTan points to right
            if (endTan.squaredNorm() == 0)
                A(Eigen::all, Eigen::last) = ddBases(Eigen::all, Eigen::last);
            else
                A(Eigen::all, Eigen::last) = dBases(Eigen::all, Eigen::last),
                              rhs(Eigen::all, Eigen::last) = endTan;
        }
        A.transposeInPlace(); // * A=A^T
        auto decomp = A.fullPivLu();
        if (decomp.isInvertible())
            b_points = decomp.solve(rhs.transpose()).transpose();
        else
        {
            auto decompSVD = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
            b_points = decompSVD.solve(rhs.transpose()).transpose();
        }
    }

}