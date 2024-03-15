//
// Created by homwen on 8/4/22.
//

#include "string_utils.hpp"


void split_string(const std::string& src_string, std::vector<std::string>& split_result,
                  const std::string& sep)
{
    split_result.clear();
    std::string::size_type pos1, pos2;
    pos2 = src_string.find(sep);
    pos1 = 0;
    while (std::string::npos != pos2) {
        split_result.push_back(src_string.substr(pos1, pos2 - pos1));

        pos1 = pos2 + sep.size();
        pos2 = src_string.find(sep, pos1);
    }
    if (pos1 != src_string.length())
        split_result.push_back(src_string.substr(pos1));
}

std::string get_path(const std::vector<std::string>& paths,
                          const std::string& suffix)
{
    std::string the_path;
    for (const auto& one_path : paths) {
        auto dot_pos = one_path.rfind('.');
        auto tmp_suffix = one_path.substr(dot_pos + 1);
        if (tmp_suffix == suffix) {
            the_path = one_path;
        }
    }
    return the_path;
}

std::string get_basename(const std::string& path)
{
    return path.substr(path.find_last_of("/") + 1);
}
