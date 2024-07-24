#pragma once

#include "dwgsimDefs.h"
#include <Eigen/Dense>

namespace DwgSim
{
    Eigen::MatrixXd getEigenMatrixFromCSV(std::istream &i)
    {

        std::vector<std::vector<double>> rows;
        rows.reserve(16);

        int ncol = 0;

        std::string line;
        while (!i.eof())
        {
            std::getline(i, line);
            if (!line.size())
                continue;
            std::stringstream ss(std::move(line));
            std::vector<double> row;
            row.reserve(ncol);
            while (!ss.eof())
            {
                std::string entry;
                std::getline(ss, entry, ',');
                if (entry.size())
                {
                    try
                    {
                        size_t idx{0};
                        auto v = std::stod(entry.c_str(), &idx);
                        row.push_back(v);
                    }
                    catch (const std::invalid_argument &e)
                    {
                        // do nothing
                    }
                }
            }
            if (ncol == 0)
                ncol = int(row.size());
            else if (ncol != row.size())
                throw std::runtime_error("csv input row size bad!");
            rows.push_back(std::move(row));
        }
        Eigen::MatrixXd ret;
        ret.resize(rows.size(), ncol);
        for (int i = 0; i < rows.size(); i++)
            for (int j = 0; j < ncol; j++)
                ret(i, j) = rows.at(i).at(j);
        return ret;
    }
}