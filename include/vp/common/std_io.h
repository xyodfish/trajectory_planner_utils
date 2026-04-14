/**
 * @file std_io.h
 * @author original.author@example.com
 * @brief 
 * @version 0.1
 * @date 2025-01-13
 * 
 * @copyright Copyright (c) 2025 Original Source
 */

#ifndef STD_IO_H_
#define STD_IO_H_

#include <fstream>
#include <string>
#include <vector>

/**
 * @brief 读取 std::vector<std::vector<double>>
 * @param data 
 * @param file_name 
 * @param cols 
 */
template <typename T>
void readSTDVector(std::vector<std::vector<T>>& data, std::string file_name, int cols) {
    std::ifstream f;
    f.open(file_name);

    std::vector<T> data_temp(cols);
    data.clear();

    while (f.peek() != EOF) {
        for (int i = 0; i < cols; i++) {
            f >> data_temp[i];
        }
        data.push_back(data_temp);
    }
};

#endif