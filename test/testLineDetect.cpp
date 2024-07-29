
#include "lineDetect.h"
#include "csvUtil.h"
#include <cassert>
#include <fstream>

namespace DwgSim
{
    void test1()
    {
        t_eigenPts<6> lineSet;
        lineSet.push_back(Eigen::Vector<double, 6>{0, 0, 0, 0, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{0, 0, 0, 1, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{0, 1, 0, 0, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{0, 1, 0, 0, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{0, 0, 0, 0, 0, 0});
        auto ret = getPtsDuplications<6>(lineSet, 1e-5);
        for (auto &s : ret)
        {
            for (auto &v : s)
                std::cout << v << ",";
            std::cout << std::endl;
        }
        assert(ret.size() == 2);
    }

    void test2()
    {
        t_eigenPts<6> lineSet;
        lineSet.push_back(Eigen::Vector<double, 6>{0, 0, 0, 0, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{0, 0, 0, 1, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{1, 0, 0, 0, 1, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{1, 0, 0, -1, 2, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{-2, 3, 0, 2, -1, 0});
        auto ret = getLinesDuplicationsAtInf(lineSet, 1e-8);
        for (auto &s : ret)
        {
            for (auto &v : s)
                std::cout << v << ",";
            std::cout << std::endl;
        }
        assert(ret.size() == 2);
    }

    void test3()
    {
        t_eigenPts<6> lineSet;
        lineSet.push_back(Eigen::Vector<double, 6>{0, 0, 0, 0, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{0, 0, 0, 1, 0, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{1, 0, 0, 0, 1, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{1, 0, 0, -1, 2, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{-2, 3, 0, 2, -1, 0});
        lineSet.push_back(Eigen::Vector<double, 6>{1, 0, 0, 0, 1, 0});
        auto [retPrecise, retInclude] = linesDuplications(lineSet, 1e-8, 1e-5);
        for (auto &s : retPrecise)
        {
            for (auto &v : s)
                std::cout << v << ",";
            std::cout << std::endl;
        }
        for (auto &s : retInclude)
        {
            std::cout << s.first << " C= " << s.second << std::endl;
        }
        assert(retPrecise.size() == 1);
        assert(retInclude.size() == 3);
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Test1: " << std::endl;
    DwgSim::test1();
    std::cout << "Test2: " << std::endl;
    DwgSim::test2();
    std::cout << "Test3: " << std::endl;
    DwgSim::test3();
    return 0;
}