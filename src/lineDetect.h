#pragma once

#include "dwgsimDefs.h"
#include "splineUtil.h"
#include <set>
#include <vector>
DISABLE_WARNING_PUSH
#if defined(_MSC_VER) && defined(_WIN32) && !defined(__clang__)
#pragma warning(disable : 4267)
#endif
#include <nanoflann.hpp>
DISABLE_WARNING_POP
#include <Eigen/Dense>

namespace DwgSim
{
    template <
        class VectorOfVectorsType, typename num_t = double, int DIM = -1,
        class Distance = nanoflann::metric_L2, typename IndexType = size_t>
    struct KDTreeVectorOfVectorsAdaptor
    {
        using self_t = KDTreeVectorOfVectorsAdaptor<
            VectorOfVectorsType, num_t, DIM, Distance, IndexType>;
        using metric_t =
            typename Distance::template traits<num_t, self_t>::distance_t;
        using index_t =
            nanoflann::KDTreeSingleIndexAdaptor<metric_t, self_t, DIM, IndexType>;

        /** The kd-tree index for the user to call its methods as usual with any
         * other FLANN index */
        index_t *index = nullptr;

        /// Constructor: takes a const ref to the vector of vectors object with the
        /// data points
        KDTreeVectorOfVectorsAdaptor(
            const size_t /* dimensionality */, const VectorOfVectorsType &mat,
            const int leaf_max_size = 10, const unsigned int n_thread_build = 1)
            : m_data(mat)
        {
            assert(mat.size() != 0 && mat[0].size() != 0);
            const size_t dims = mat[0].size();
            if (DIM > 0 && static_cast<int>(dims) != DIM)
                throw std::runtime_error(
                    "Data set dimensionality does not match the 'DIM' template "
                    "argument");
            index = new index_t(
                static_cast<int>(dims), *this /* adaptor */,
                nanoflann::KDTreeSingleIndexAdaptorParams(
                    leaf_max_size, nanoflann::KDTreeSingleIndexAdaptorFlags::None,
                    n_thread_build));
        }

        ~KDTreeVectorOfVectorsAdaptor() { delete index; }

        const VectorOfVectorsType &m_data;

        /** Query for the \a num_closest closest points to a given point
         *  (entered as query_point[0:dim-1]).
         *  Note that this is a short-cut method for index->findNeighbors().
         *  The user can also call index->... methods as desired.
         */
        inline void query(
            const num_t *query_point, const size_t num_closest,
            IndexType *out_indices, num_t *out_distances_sq) const
        {
            nanoflann::KNNResultSet<num_t, IndexType> resultSet(num_closest);
            resultSet.init(out_indices, out_distances_sq);
            index->findNeighbors(resultSet, query_point);
        }

        /** @name Interface expected by KDTreeSingleIndexAdaptor
         * @{ */

        const self_t &derived() const { return *this; }
        self_t &derived() { return *this; }

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return m_data.size(); }

        // Returns the dim'th component of the idx'th point in the class:
        inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            return m_data[idx][dim];
        }

        // Optional bounding-box computation: return false to default to a standard
        // bbox computation loop.
        // Return true if the BBOX was already computed by the class and returned
        // in "bb" so it can be avoided to redo it again. Look at bb.size() to
        // find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX & /*bb*/) const
        {
            return false;
        }

        /** @} */

    }; // end of KDTreeVectorOfVectorsAdaptor

    template <int dim>
    using t_eigenPts = std::vector<Eigen::Vector<double, dim>>;

    template <int dim>
    inline auto getPtsDuplications(t_eigenPts<dim> &linesInf, double eps)
    {
        using namespace nanoflann;
        typedef KDTreeVectorOfVectorsAdaptor<t_eigenPts<dim>, double>
            kd_tree_t;

        std::vector<std::set<int64_t>> ret;
        if (!linesInf.size())
            return ret;

        kd_tree_t kd_tree(linesInf[0].size(), linesInf);

        std::vector<int64_t> inSet;
        inSet.resize(linesInf.size(), -1);

        nanoflann::SearchParameters params;
        params.sorted = true;
        for (int i = 0; i < linesInf.size(); i++)
        {
            std::vector<nanoflann::ResultItem<size_t, double>> result;
            kd_tree.index->radiusSearch(linesInf[i].data(), (eps * eps), result, params);
            if (result.size() > 1) // has to contain itself
            {
                if (inSet[i] == -1)
                {
                    inSet[i] = static_cast<int64_t>(ret.size());
                    ret.emplace_back();
                }
                auto &cset = ret.at(inSet[i]);
                cset.insert(i);
                for (auto j : result)
                {
                    cset.insert(static_cast<int64_t>(j.first));
                    inSet[j.first] = inSet[i];
                }
            }
        }

        return ret;
    }

    template <int dim>
    inline auto getPtsDuplicationsInPts(t_eigenPts<dim> &linesInf, t_eigenPts<dim> &tests, double eps)
    {
        using namespace nanoflann;
        typedef KDTreeVectorOfVectorsAdaptor<t_eigenPts<dim>, double>
            kd_tree_t;

        std::vector<std::pair<int64_t, int64_t>> ret;
        if (!linesInf.size())
            return ret;

        kd_tree_t kd_tree(dim, linesInf);

        nanoflann::SearchParameters params;
        params.sorted = true;
        for (int64_t i = 0; i < int64_t(tests.size()); i++)
        {
            std::vector<nanoflann::ResultItem<size_t, double>> result;
            kd_tree.index->radiusSearch(tests[i].data(), (eps * eps), result, params);
            for (auto j : result)
                ret.push_back(std::make_pair(int64_t(j.first), i));
        }
        return ret;
    }

    static auto Seq012 = Eigen::seq(Eigen::fix<0>, Eigen::fix<2>);
    static auto Seq345 = Eigen::seq(Eigen::fix<3>, Eigen::fix<5>);
    static auto Seq678 = Eigen::seq(Eigen::fix<6>, Eigen::fix<8>);

    /**
     * @brief
     *
     * @param lines vector of (x1 y1 z1 x2 y2 z2)
     * @return vector of (xd yd zd xb yb zb), d for direction, b for base point
     */
    inline auto linesToInfLine(t_eigenPts<6> &lines)
    {
        t_eigenPts<6> ret = lines;
        for (int64_t i = 0; i < (int64_t)lines.size(); i++)
        {
            Vec3 p0 = lines[i](Seq012);
            Vec3 p1 = lines[i](Seq345);
            double LRef = std::max(p0.norm(), p1.norm());
            Vec3 dir;
            Vec3 base;
            dir = p1 - p0;
            double dirNormSqr = dir.squaredNorm();
            if (dir.norm() < LRef * 1e-10 || LRef == 0)
            {
                dir.setZero();
                dir(0) = 1;
                base = p0;
            }
            else
            {
                double alphaB = -p0.dot(dir) / (dirNormSqr + 1e-300);
                base = (1 - alphaB) * p0 + alphaB * p1;
            }
            dir.normalize();
            if (std::abs(dir(2)) > 1e-13) //! discard dir diff
            {
                if (dir(2) < 0)
                    dir *= -1;
            }
            else if (std::abs(dir(0)) > 1e-13)
            {
                if (dir(0) < 0)
                    dir *= -1;
            }
            else
            {
                if (dir(1) < 0)
                    dir *= -1;
            }

            ret[i](Seq012) = dir;
            ret[i](Seq345) = base;
        }

        return ret;
    }

    inline auto getInfLineNormalized(t_eigenPts<6> &linesInf, double &maxBaseSiz, bool updateMax = true)
    {
        t_eigenPts<6> ret = linesInf;
        if (updateMax)
            maxBaseSiz = 1e-300;
        if (updateMax)
            for (int64_t i = 0; i < (int64_t)linesInf.size(); i++)
                maxBaseSiz = std::max(maxBaseSiz, ret[i](Seq345).norm());
        for (int64_t i = 0; i < (int64_t)linesInf.size(); i++)
            ret[i](Seq345) /= maxBaseSiz;
        return ret;
    }

    inline auto getLinesDuplicationsAtInf(t_eigenPts<6> &lines, double eps = 1e-8)
    {
        double maxBase;
        auto linesInf = linesToInfLine(lines);
        auto linesInfNorm = getInfLineNormalized(linesInf, maxBase);
        return getPtsDuplications<6>(linesInfNorm, eps);
    }

    /**
     * @brief
     *
     * @param lines vector of (x1 y1 z1 x2 y2 z2)
     * @return (preciseDups, includeDups)
     */
    inline auto linesDuplications(t_eigenPts<6> &lines, double eps = 1e-8, double lEps = 1e-5)
    {
        double maxBase{1e-100};
        auto linesInf = linesToInfLine(lines);
        auto linesInfNorm = getInfLineNormalized(linesInf, maxBase);
        auto infDup = getPtsDuplications<6>(linesInfNorm, eps);
        std::vector<int64_t> inPreciseDups(lines.size(), -1);
        std::vector<std::set<int64_t>> preciseDups;
        std::vector<std::pair<int64_t, int64_t>> includeDups;

        for (auto &s : infDup)
        {
            for (auto i : s)
            {
                Vec3 dir = linesInf[i](Seq012);
                Vec3 base = linesInf[i](Seq345);
                double Li = (lines[i](Seq012) - base).dot(dir);
                double Ri = (lines[i](Seq345) - base).dot(dir);
                if (Li > Ri)
                    std::swap(Li, Ri);
                for (auto j : s) //! currently use O(N^2) search
                {
                    if (i == j)
                        continue;
                    double Lj = (lines[j](Seq012) - base).dot(dir);
                    double Rj = (lines[j](Seq345) - base).dot(dir);
                    if (Lj > Rj)
                        std::swap(Lj, Rj);

                    if (std::abs(Li - Lj) < lEps && std::abs(Ri - Rj) < lEps)
                    {
                        if (inPreciseDups[i] == -1)
                        {
                            inPreciseDups[i] = preciseDups.size();
                            preciseDups.emplace_back();
                        }
                        inPreciseDups[j] = inPreciseDups[i];
                        preciseDups.at(inPreciseDups[i]).insert(i);
                        preciseDups.at(inPreciseDups[i]).insert(j);
                    }
                    if (Li - lEps < Lj && Ri + lEps > Rj)
                        includeDups.emplace_back(std::make_pair(i, j));
                }
            }
        }

        return std::make_tuple(preciseDups, includeDups);
    }

    inline auto lineInLinesDuplications(t_eigenPts<6> &lines, t_eigenPts<6> &linesTest, double eps = 1e-8, double lEps = 1e-5)
    {
        double maxL{1e-100};
        auto linesInf = linesToInfLine(lines);
        auto linesInfNorm = getInfLineNormalized(linesInf, maxL);
        auto linesInfTest = linesToInfLine(linesTest);
        auto linesInfNormTest = getInfLineNormalized(linesInfTest, maxL, false);

        auto infDup = getPtsDuplicationsInPts<6>(linesInfNorm, linesInfNormTest, eps);

        std::vector<std::pair<int64_t, int64_t>> preciseDups;
        std::vector<std::pair<int64_t, int64_t>> includeDups;

        for (auto &p : infDup)
        {
            auto i = p.first;
            auto j = p.second;

            Vec3 dir = linesInf[i](Seq012);
            Vec3 base = linesInf[i](Seq345);
            double Li = (lines[i](Seq012) - base).dot(dir);
            double Ri = (lines[i](Seq345) - base).dot(dir);
            if (Li > Ri)
                std::swap(Li, Ri);
            double Lj = (linesTest[j](Seq012) - base).dot(dir);
            double Rj = (linesTest[j](Seq345) - base).dot(dir);
            if (Lj > Rj)
                std::swap(Lj, Rj);

            if (std::abs(Li - Lj) < lEps && std::abs(Ri - Rj) < lEps)
                preciseDups.push_back(std::make_pair(i, j));
            if (Li - lEps < Lj && Ri + lEps > Rj)
                includeDups.emplace_back(std::make_pair(i, j));
        }
        return std::make_tuple(preciseDups, includeDups);
    }

    inline auto arcsRegulate(t_eigenPts<9> &arcs, double &maxPos, double &maxR, double eps, bool updateMax = true)
    {
        t_eigenPts<9> ret = arcs;
        if (updateMax)
        {
            maxPos = 1e-100;
            maxR = 1e-100;
        }
        for (auto &v : ret)
        {
            if (v(8) < v(7))
                v(8) += 2 * pi;
            if (v(7) >= 2 * pi - eps)
                v(8) -= 2 * pi, v(7) -= 2 * pi;
            assert(v(8) >= v(7));

            v(Seq012).normalize();
            if (updateMax)
            {
                maxPos = std::max(maxPos, v(Seq345).norm());
                maxR = std::max(maxR, v(6));
            }
        }
        for (auto &v : ret)
        {
            v(Seq345) /= maxPos;
            v(6) /= maxR;
        }
        return ret;
    }

    inline auto arcsToCircle(t_eigenPts<9> &arcs)
    {
        t_eigenPts<7> ret;
        for (auto &v : arcs)
            ret.push_back(v(Eigen::seq(0, 6)));
        return ret;
    }

    inline auto arcsDuplications(t_eigenPts<9> &arcs, double eps = 1e-8)
    {
        double maxPos{1e-100}, maxR{1e-100};
        auto arcsReg = arcsRegulate(arcs, maxPos, maxR, eps);
        auto circs = arcsToCircle(arcsReg);
        // std::vector<int64_t> inPreciseDups(arcs.size(), -1);
        std::vector<std::set<int64_t>> preciseDups;
        std::vector<std::pair<int64_t, int64_t>> includeDups;
        auto circDup = getPtsDuplications<7>(circs, eps);
        preciseDups = getPtsDuplications<9>(arcsReg, eps);

        for (auto &s : circDup)
        {
            for (auto i : s)
            {
                double t0 = arcsReg[i](7);
                double t1 = arcsReg[i](8);
                for (auto j : s)
                {
                    if (j == i)
                        continue;
                    double t0c = arcsReg[j](7);
                    double t1c = arcsReg[j](8);
                    if (t0 <= t0c + eps && t1 >= t1c - eps)
                        includeDups.push_back(std::make_pair(i, j));
                    if (t0 == 0 && t1 == 2 * pi) // is a circle
                        includeDups.push_back(std::make_pair(i, j));
                }
            }
        }

        return std::make_tuple(preciseDups, includeDups);
    }

    inline auto arcInArcsDuplications(t_eigenPts<9> &arcs, t_eigenPts<9> &arcsTests, double eps = 1e-8)
    {
        double maxPos{1e-100}, maxR{1e-100};
        auto arcsReg = arcsRegulate(arcs, maxPos, maxR, eps);
        auto circs = arcsToCircle(arcsReg);

        auto arcsRegTests = arcsRegulate(arcsTests, maxPos, maxR, eps, false);
        auto circsTests = arcsToCircle(arcsRegTests);

        auto circDup = getPtsDuplicationsInPts<7>(circs, circsTests, eps);
        auto preciseDups = getPtsDuplicationsInPts<9>(arcsReg, arcsRegTests, eps);

        std::vector<std::pair<int64_t, int64_t>> includeDups;
        for (auto &p : circDup)
        {
            auto i = p.first;
            auto j = p.second;

            double t0 = arcsReg[i](7);
            double t1 = arcsReg[i](8);
            double t0c = arcsRegTests[j](7);
            double t1c = arcsRegTests[j](8);
            if (t0 <= t0c + eps && t1 >= t1c - eps)
                includeDups.push_back(std::make_pair(i, j));
            if (t0 == 0 && t1 == 2 * pi) // is a circle
                includeDups.push_back(std::make_pair(i, j));
        }

        return std::make_tuple(preciseDups, includeDups);
    }

    /**
     * @brief
     *
     * @param L
     * @param R
     * @param eps
     * @return int (-1 if L < R), (1 if L > R), (0 if L = R)
     */
    inline int coordCompare(const Vec3 &L, const Vec3 &R, double eps)
    {
        for (int dim = 0; dim < 2; dim++)
        {
            if (L(dim) < R(dim) - eps)
                return -1;
            else if (L(dim) - eps > R(dim))
                return 1;
        }
        return 0;
    }

    class PolylineGeomSet
    {
        std::map<int, std::set<int64_t>> siz_to_idx;
        std::vector<int64_t> poly2list;
        std::vector<Eigen::VectorXd> polyVecs; // size of vectorxd is p_siz * 4 + 3
    public:
        void insertPoly(int64_t list_idx, int siz, const Eigen::VectorXd &polyData)
        {
            poly2list.push_back(list_idx);
            polyVecs.push_back(polyData);
            if (!siz_to_idx.count(siz))
                siz_to_idx[siz] = std::set<int64_t>();
            siz_to_idx.at(siz).insert(poly2list.size() - 1);
        }

        auto getDuplicates(double eps = 1e-8)
        {
            std::vector<std::set<int64_t>> ret;
            for (auto &[siz, s] : siz_to_idx)
            {
                std::vector<Eigen::VectorXd> polyVecsCur;
                std::vector<int64_t> localIdx;
                polyVecsCur.reserve(s.size());
                localIdx.reserve(s.size());
                double maxL{1e-100};
                for (auto i : s)
                {
                    polyVecsCur.push_back(polyVecs[i]);
                    localIdx.push_back(i);
                    for (int ii = 0; ii < siz; ii++)
                        maxL = std::max(maxL, polyVecs[i](Eigen::seq(3 + 4 * ii, 5 + 4 * ii)).array().abs().maxCoeff());
                }
                for (auto &v : polyVecsCur)
                {
                    for (int ii = 0; ii < siz; ii++)
                        v(Eigen::seq(3 + 4 * ii, 5 + 4 * ii)) /= maxL; // normalize
                    v(v.size() - 1) = 0;                               // last bulge is not used
                    if (siz >= 2)
                    {
                        int cmp01 = coordCompare(v(Eigen::seq(3 + 4 * 0, 5 + 4 * 0)), v(Eigen::seq(3 + 4 * (siz - 1), 5 + 4 * (siz - 1))), eps);
                        bool invert = false;
                        if (cmp01 > 0) 
                            invert = true;
                        if (cmp01 == 0)
                        {
                            int cmpm11 = coordCompare(v(Eigen::seq(3 + 4 * 1, 5 + 4 * 1)), v(Eigen::seq(3 + 4 * (siz - 2), 5 + 4 * (siz - 2))), eps);
                            if (cmpm11 > 0 || (cmpm11 == 0 && v(6 + 4 * 0) < 0) || (cmpm11 == 0 && v(6 + 4 * (siz - 2)) < 0))
                                invert = true;
                        }
                        if (invert) // invert
                        {
                            Eigen::VectorXd v1 = v;
                            for (int ii = 0; ii < siz; ii++)
                                v1(Eigen::seq(3 + 4 * ii, 5 + 4 * ii)) = v(Eigen::seq(3 + 4 * (siz - 1 - ii), 5 + 4 * (siz - 1 - ii)));
                            for (int ii = 0; ii < siz - 1; ii++)
                                v1(6 + 4 * ii) = -v(6 + 4 * (siz - 2 - ii));
                            v = v1;
                        }
                    }
                }

                auto dups = getPtsDuplications<Eigen::Dynamic>(polyVecsCur, eps);
                for (auto &ss : dups)
                {
                    ret.emplace_back();
                    for (auto ii : ss)
                        ret.back().insert(poly2list[localIdx[ii]]);
                }
            }
            return ret;
        }
    };
}