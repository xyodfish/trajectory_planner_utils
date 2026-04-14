#ifndef LOAD_YAML_H
#define LOAD_YAML_H
#include <fstream>
#include <iostream>
#include <string>

#include <yaml-cpp/yaml.h>

template <typename T>
void save_yaml(const std::string& file_addr, const std::string& key, T value) {
    YAML::Node status_node = YAML::LoadFile(file_addr);

    status_node[key] = value;

    YAML::Emitter emmiter;
    emmiter.SetSeqFormat(YAML::Flow);
    emmiter << status_node;

    std::ofstream dump_file(file_addr);
    dump_file << emmiter.c_str();
    dump_file.close();
}

template <typename T>
T load_yaml(const std::string& file_addr, const std::string& key) {
    YAML::Node status_node = YAML::LoadFile(file_addr);
    return status_node[key].as<T>();
}

inline YAML::Node load_yaml_node(const YAML::Node& node, const std::string& key) {
    if (!node[key]) {
        std::cerr << "yaml file has no key: " << key << std::endl;
    }
    return node[key];
}

template <typename T>
T load_yaml(const YAML::Node& node, const std::string& key) {
    if (!node[key]) {
        std::cerr << "yaml file has no key: " << key << std::endl;
    }
    return node[key].as<T>();
}

#endif  // !LOAD_YAML_H
