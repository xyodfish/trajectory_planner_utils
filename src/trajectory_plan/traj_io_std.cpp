/**
 * @file traj_io_std.cpp
 * @author original.author@example.com
 * @brief 
 * @version 0.1
 * @date 2024-12-17
 * 
 * @copyright Copyright (c) 2024 Original Source
 */

#include "traj_io_std.h"
#include "std_io.h"

#include <fstream>
#include <iomanip>

namespace vp::tp {
    void readPath(std::vector<std::vector<double>>& path, const std::string& file_name) {
        readSTDVector(path, file_name, 6);
    }

    void readTrajLSTD(std::vector<std::vector<double>>& traj, std::string file_name, int dim, int differential_order) {
        int size = (differential_order + 1) * dim + 1;
        readSTDVector(traj, file_name, size);
    }

    void readTrajLTransSTD(std::vector<std::vector<double>>& traj, std::string file_name, int dim,
                           int differential_order) {

        std::ifstream f;
        f.open(file_name);

        int size = (differential_order + 1) * dim + 1;

        traj.clear();
        traj.resize(size);

        double val;
        while (f.peek() != EOF) {
            for (int i = 0; i < size; i++) {
                f >> val;
                traj[i].push_back(val);
            }
        }
    }

    void writeTrajLSTD(const std::vector<std::vector<double>>& traj, const std::string& file,
                       const std::string& separator) {
        unsigned int num = traj.size();
        unsigned int dim = traj[0].size();

        std::ofstream f;
        f.open(file);

        f << std::setprecision(9);

        for (unsigned int i = 0; i < num; i++) {

            for (unsigned int j = 0; j < dim; j++) {
                f << traj[i][j];
                if (j != dim - 1) {
                    f << separator;
                }
            }

            if (i != num - 1) {
                f << '\n';
            }
        }

        f.close();
    }

    void writeTrajLTransSTD(const std::vector<std::vector<double>>& traj, const std::string& file,
                            const std::string& separator) {
        unsigned int num = traj[0].size();
        unsigned int dim = traj.size();

        std::ofstream f;
        f.open(file);

        f << std::setprecision(9);

        for (unsigned int i = 0; i < num; i++) {

            for (unsigned int j = 0; j < dim; j++) {
                f << traj[j][i];
                if (j != dim - 1) {
                    f << separator;
                }
            }

            if (i != num - 1) {
                f << '\n';
            }
        }

        f.close();
    }

    void trajTrans(const std::vector<std::vector<double>>& src, std::vector<std::vector<double>>& dst) {
        int dim_1 = src.size();
        int dim_2 = src[0].size();

        dst.clear();
        dst.resize(dim_2);

        std::vector<double> data(dim_1);

        for (int i = 0; i < dim_2; i++) {
            for (int j = 0; j < dim_1; j++) {
                data[j] = src[j][i];
            }
            dst[i] = data;
        }
    }
}  // namespace vp::tp