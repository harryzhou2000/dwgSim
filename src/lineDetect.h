#pragma once

#include "dwgsimDefs.h"
#include "splineUtil.h"
#include <set>
#include <vector>
#include <nanoflann.hpp>
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

        kd_tree_t kd_tree(dim, linesInf);

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

    static auto Seq012 = Eigen::seq(Eigen::fix<0>, Eigen::fix<2>);
    static auto Seq345 = Eigen::seq(Eigen::fix<3>, Eigen::fix<5>);

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

    inline auto getInfLineNormalized(t_eigenPts<6> &linesInf)
    {
        t_eigenPts<6> ret = linesInf;
        double maxBaseSiz = 1e-300;
        for (int64_t i = 0; i < (int64_t)linesInf.size(); i++)
            maxBaseSiz = std::max(maxBaseSiz, ret[i](Seq345).norm());
        for (int64_t i = 0; i < (int64_t)linesInf.size(); i++)
            ret[i](Seq345) /= maxBaseSiz;
        return ret;
    }

    inline auto getLinesDuplicationsAtInf(t_eigenPts<6> &lines, double eps = 1e-8)
    {
        auto linesInf = linesToInfLine(lines);
        auto linesInfNorm = getInfLineNormalized(linesInf);
        return getPtsDuplications<6>(linesInfNorm, eps);
    }

    inline auto linesDuplications(t_eigenPts<6> &lines, double eps = 1e-8, double lEps = 1e-5)
    {
        auto linesInf = linesToInfLine(lines);
        auto linesInfNorm = getInfLineNormalized(linesInf);
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
}