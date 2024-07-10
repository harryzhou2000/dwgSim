
#include "splineUtil.h"
#include <cassert>

namespace DwgSim
{
    void test1()
    {
        VecX knots;
        knots.resize(6);
        knots << 0, 1, 2, 3, 4, 5;
        knots *= 100;
        Mat3X pts;
        pts.resize(3, 6);
        pts(Eigen::all, 0) = Vec3{0, 0, 0};
        pts(Eigen::all, 1) = Vec3{0.0000000000000061, 100, 0};
        pts(Eigen::all, 2) = Vec3{100, 100, 0};
        pts(Eigen::all, 3) = Vec3{100, 0, 0};
        pts(Eigen::all, 4) = Vec3{200, 0, 0};
        pts(Eigen::all, 5) = Vec3{200, 100, 0};

        VecX b_knots;
        Mat3X b_pts;

        DwgSim::CubicSplineToBSpline(
            knots, pts, Vec3{0, 0, 0}, Vec3{0, 0, 0},
            b_knots, b_pts);

        // std::cout << "b_pts" << std::endl;
        // std::cout << b_pts << std::endl;

        Mat3X b_pts_ans;
        b_pts_ans.resize(3, 6 + 2);
        b_pts_ans(Eigen::all, 0) = Vec3{0, 0, 0};
        b_pts_ans(Eigen::all, 1) = Vec3{-12.12121212121211, 39.39393939393939, 0};
        b_pts_ans(Eigen::all, 2) = Vec3{-36.36363636363635, 118.1818181818181, 0};
        b_pts_ans(Eigen::all, 3) = Vec3{145.4545454545454, 127.2727272727272, 0};
        b_pts_ans(Eigen::all, 4) = Vec3{54.5454545454545, -27.27272727272727, 0};
        b_pts_ans(Eigen::all, 5) = Vec3{236.3636363636364, -18.18181818181819, 0};
        b_pts_ans(Eigen::all, 6) = Vec3{212.1212121212121, 60.60606060606061, 0};
        b_pts_ans(Eigen::all, 7) = Vec3{200.0, 100, 0};
        double err = (b_pts_ans - b_pts).norm();
        assert(err < 1e-10);
        std::cout << "Test Error: " << err << std::endl;
    }

    void test2()
    {
        // VecX knots;
        // knots.resize(3);
        // knots << 70.71067811865474, 141.4213562373095, 2, 3, 4, 5;
        // knots *= 100;
        // Mat3X pts;
        // pts.resize(3, 6);
        // pts(Eigen::all, 0) = Vec3{0, 0, 0};
        // pts(Eigen::all, 1) = Vec3{0.0000000000000061, 100, 0};
        // pts(Eigen::all, 2) = Vec3{100, 100, 0};
        // pts(Eigen::all, 3) = Vec3{100, 0, 0};
        // pts(Eigen::all, 4) = Vec3{200, 0, 0};
        // pts(Eigen::all, 5) = Vec3{200, 100, 0};

        // VecX b_knots;
        // Mat3X b_pts;

        // DwgSim::CubicSplineToBSpline(
        //     knots, pts, Vec3{0, 0, 0}, Vec3{0, 0, 0},
        //     b_knots, b_pts);

        // // std::cout << "b_pts" << std::endl;
        // // std::cout << b_pts << std::endl;

        // Mat3X b_pts_ans;
        // b_pts_ans.resize(3, 6 + 2);
        // b_pts_ans(Eigen::all, 0) = Vec3{0, 0, 0};
        // b_pts_ans(Eigen::all, 1) = Vec3{-12.12121212121211, 39.39393939393939, 0};
        // b_pts_ans(Eigen::all, 2) = Vec3{-36.36363636363635, 118.1818181818181, 0};
        // b_pts_ans(Eigen::all, 3) = Vec3{145.4545454545454, 127.2727272727272, 0};
        // b_pts_ans(Eigen::all, 4) = Vec3{54.5454545454545, -27.27272727272727, 0};
        // b_pts_ans(Eigen::all, 5) = Vec3{236.3636363636364, -18.18181818181819, 0};
        // b_pts_ans(Eigen::all, 6) = Vec3{212.1212121212121, 60.60606060606061, 0};
        // b_pts_ans(Eigen::all, 7) = Vec3{200.0, 100, 0};
        // double err = (b_pts_ans - b_pts).norm();
        // assert(err < 1e-10);
        // std::cout << "Test Error: " << err << std::endl;
    }
}

int main()
{
    DwgSim::test1();
    return 0;
}