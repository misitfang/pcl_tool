//
// Created by homwen on 8/4/22.
//
#ifndef __DETECT_COMMON_CPP_STRING_UTILS_HPP__
#define __DETECT_COMMON_CPP_STRING_UTILS_HPP__
#include <string>
#include <vector>

void split_string(const std::string& src_string, std::vector<std::string>& split_result,
                  const std::string& sep);

std::string get_path(const std::vector<std::string>& paths, const std::string& suffix);

std::string get_basename(const std::string& path);

template <typename... Args> std::string format_string(const std::string& format, Args... args)
{
    // Extra space for \0
    size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args...);
    // unique_ptr<char[]> buf(new char[size]);
    char bytes[size];
    snprintf(bytes, size, format.c_str(), args...);
    return std::string(bytes);
}



#endif //
