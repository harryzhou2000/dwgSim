
#include "splineUtil.h"
#include "csvUtil.h"
#include <cassert>
#include <fstream>

namespace DwgSim
{
    static std::string dataDir;
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
        double err = (b_pts_ans - b_pts).norm() / b_pts_ans.norm();
        assert(err < 1e-10);
        std::cout << "Test Error: " << err << std::endl;
    }

    void test2()
    {
        VecX knots;
        knots.resize(4);
        knots << 0, 70.71067811865474, 141.4213562373095, 241.4213562373095;
        Mat3X pts;
        pts.resize(3, 4);
        pts(Eigen::all, 0) = Vec3{0, 0, 0};
        pts(Eigen::all, 1) = Vec3{-50, -50, 0};
        pts(Eigen::all, 2) = Vec3{0, -100, 0};
        pts(Eigen::all, 3) = Vec3{0, 0, 0};

        VecX b_knots;
        Mat3X b_pts;

        DwgSim::CubicSplineToBSpline(
            knots, pts, Vec3{0, 0, 0}, Vec3{0, 0, 0},
            b_knots, b_pts, true);

        // std::cout << "b_pts" << std::endl;
        // std::cout << b_pts.transpose() << std::endl;

        Mat3X b_pts_ans;
        b_pts_ans.resize(3, 4 + 2);
        b_pts_ans(Eigen::all, 0) = Vec3{0, -0.0000000000000009, 0};
        b_pts_ans(Eigen::all, 1) = Vec3{-18.46990312590645, 6.90355937288492, 0};
        b_pts_ans(Eigen::all, 2) = Vec3{-81.53009687409351, -49.99999999999997, 0};
        b_pts_ans(Eigen::all, 3) = Vec3{26.12038749637413, -147.1404520791032, 0};
        b_pts_ans(Eigen::all, 4) = Vec3{26.12038749637414, -9.763107293781757, 0};
        b_pts_ans(Eigen::all, 5) = Vec3{0, -0.0000000000000009, 0};

        double err = (b_pts_ans - b_pts).norm() / b_pts_ans.norm();
        assert(err < 1e-10);
        std::cout << "Test Error: " << err << std::endl;
    }

    void test3()
    {
        VecX knots;
        knots.resize(4);
        knots << 0, 150.0, 308.1138830084189, 378.8245611270737;
        Mat3X pts;
        pts.resize(3, 4);
        pts(Eigen::all, 0) = Vec3{0, 0, 0};
        pts(Eigen::all, 1) = Vec3{-0.0000000000000276, -150, 0};
        pts(Eigen::all, 2) = Vec3{150.0, -100, 0};
        pts(Eigen::all, 3) = Vec3{100.0, -50, 0};

        VecX b_knots;
        Mat3X b_pts;

        DwgSim::CubicSplineToBSpline(
            knots, pts,
            Vec3{-0.7071067811865476, 0.7071067811865476, 0} * 1,
            Vec3{-0.7071067811865476, -0.7071067811865476, 0} * 1,
            b_knots, b_pts, false);

        // std::cout << "b_pts" << std::endl;
        // std::cout << b_pts.transpose() << std::endl;

        Mat3X b_pts_ans;
        b_pts_ans.resize(3, 4 + 2);
        b_pts_ans(Eigen::all, 0) = Vec3{0, 0, 0};
        b_pts_ans(Eigen::all, 1) = Vec3{-35.35533905932738, 35.35533905932738, 0};
        b_pts_ans(Eigen::all, 2) = Vec3{-57.29535684462918, -239.3247003470849, 0};
        b_pts_ans(Eigen::all, 3) = Vec3{209.9579208246919, -151.1854715823055, 0};
        b_pts_ans(Eigen::all, 4) = Vec3{116.6666666666666, -33.33333333333333, 0};
        b_pts_ans(Eigen::all, 5) = Vec3{100.0, -50., 0};

        double err = (b_pts_ans - b_pts).norm() / b_pts_ans.norm();
        assert(err < 1e-10);
        std::cout << "Test Error: " << err << std::endl;
    }

    void test4()
    {
        VecX knots;
        knots.resize(3);
        knots << 0, 50.0, 120.7106781186547;
        Mat3X pts;
        pts.resize(3, 3);
        pts(Eigen::all, 0) = Vec3{100.0, -50.0, 0};
        pts(Eigen::all, 1) = Vec3{50.0, -49.99999999999999, 0};
        pts(Eigen::all, 2) = Vec3{0, 0, 0};

        VecX b_knots;
        Mat3X b_pts;

        DwgSim::CubicSplineToBSpline(
            knots, pts,
            Vec3{0, 0, 0},
            Vec3{-0.7071067811865476, 0.7071067811865476, 0},
            b_knots, b_pts, false);

        // std::cout << "b_pts" << std::endl;
        // std::cout << b_pts.transpose() << std::endl;

        Mat3X b_pts_ans;
        b_pts_ans.resize(3, 3 + 2);
        b_pts_ans(Eigen::all, 0) = Vec3{100.0, -50, 0};
        b_pts_ans(Eigen::all, 1) = Vec3{82.14886980224207, -52.85954792089682, 0};
        b_pts_ans(Eigen::all, 2) = Vec3{39.05242917512703, -59.76310729378175, 0};
        b_pts_ans(Eigen::all, 3) = Vec3{16.66666666666666, -16.66666666666666, 0};
        b_pts_ans(Eigen::all, 4) = Vec3{0.0, -0., 0};

        double err = (b_pts_ans - b_pts).norm() / b_pts_ans.norm();
        assert(err < 1e-10);
        std::cout << "Test Error: " << err << std::endl;
    }

    void test5()
    {
        VecX knots;
        Mat3X pts;

        auto ispts = std::ifstream(dataDir + "/dat.csv");
        assert(ispts);
        auto ispts_read = getEigenMatrixFromCSV(ispts);
        pts.setZero(3, ispts_read.rows());
        pts({0, 1}, Eigen::all) = ispts_read.transpose();
        knots.resize(pts.cols());
        double cLen = 0;
        knots(0) = cLen;
        for (int i = 1; i < pts.cols(); i++)
            cLen += (pts(Eigen::all, i) - pts(Eigen::all, i - 1)).norm(),
                knots(i) = cLen;

        VecX b_knots;
        Mat3X b_pts;

        DwgSim::CubicSplineToBSpline(
            knots, pts,
            Vec3{0, 0, 0},
            Vec3{0, 0, 0},
            b_knots, b_pts, false);

        // std::cout << "b_pts" << std::endl;
        // std::cout << b_pts.transpose() << std::endl;

        Mat3X b_pts_ans;
        auto isb_pts = std::ifstream(dataDir + "/datAc.csv");
        assert(isb_pts);
        b_pts_ans = getEigenMatrixFromCSV(isb_pts).transpose();

        double err = (b_pts_ans - b_pts).norm() / b_pts_ans.norm();
        assert(err < 1e-10);
        std::cout << "Test Error: " << err << std::endl;
        // std::cout << "=====" << std::endl;
        // std::cout << b_pts_ans << std::endl;
        // std::cout << "=====" << std::endl;
        // std::cout << pts << std::endl;
    }
}

int main(int argc, char *argv[])
{
    assert(argc >= 2);
    DwgSim::dataDir = argv[1];

    std::cout << "Test 1: free-free" << std::endl;
    DwgSim::test1(); // free-free
    std::cout << "Test 2: periodic" << std::endl;
    DwgSim::test2(); // periodic
    std::cout << "Test 3: tan-tan" << std::endl;
    DwgSim::test3(); // tan-tan
    std::cout << "Test 4: free-tan" << std::endl;
    DwgSim::test4(); // free-tan
    std::cout << "Test 5: free-free 100 pts" << std::endl;
    DwgSim::test5();
    return 0;
}