#ifndef OPENVSLAM_UTIL_YAML_H
#define OPENVSLAM_UTIL_YAML_H

#include <string>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

namespace mvgpcgen
{
namespace util
{

    inline YAML::Node yaml_optional_ref(const YAML::Node &ref_node, const std::string &key)
    {
        return ref_node[key] ? ref_node[key] : YAML::Node();
    }

    template <typename T>
    void yaml_optional_set(const YAML::Node &node, T &out)
    {
        if (node)
        {
            try
            {
                out = node.template as<T>();
            }
            catch (std::exception &e)
            {
                fprintf(stderr, "wrong optional field: %s\n", e.what());
            }
        }
    }

}
}

#endif // OPENVSLAM_UTIL_YAML_H
