/**
 * @file sensor_config.hpp
 * @brief Shared helpers for reading the UWB sensor configuration consistently.
 *
 * The UWB emulator picks its YAML file from the noise_type argument (1=Gaussian,
 * 2=Uniform). The pose filters must read the SAME file, otherwise they build their
 * measurement model from the wrong anchor geometry and size R for the wrong noise
 * distribution. These helpers exist so both sides agree on the mapping.
 */

#ifndef INT_SYS_FP_SENSOR_CONFIG_HPP
#define INT_SYS_FP_SENSOR_CONFIG_HPP

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <string>

namespace sensor_config {

/// Sensor YAML file name (leading '/') for a given noise_type: 1=Gaussian, 2=Uniform.
inline std::string sensorYamlName(int noise_type) {
    return (noise_type == 2) ? "/config/sensor_params_uniform.yaml" : "/config/sensor_params.yaml";
}

/**
 * @brief Standard deviation to use in R for a given "noise model" YAML node.
 *
 * Gaussian: the declared stddev.
 * Uniform on [min,max]: the equivalent std is (max-min)/sqrt(12). For a symmetric
 * +/-a this is a/sqrt(3), i.e. variance a^2/3 - NOT a^2. Sizing R with `a` instead
 * would make the filter overconfident by a factor of 3 in variance.
 *
 * @param fallback returned when the node is missing or the type is unrecognised.
 */
inline double equivalentStd(const YAML::Node& noise_model, double fallback = 0.05) {
    if (!noise_model) return fallback;

    const std::string type = noise_model["type"] ?
        noise_model["type"].as<std::string>("Gaussian") : "Gaussian";

    if (type == "Uniform") {
        if (!noise_model["min"] || !noise_model["max"]) return fallback;
        const double lo = noise_model["min"].as<double>();
        const double hi = noise_model["max"].as<double>();
        const double width = hi - lo;
        if (width <= 0.0) return fallback;
        return width / std::sqrt(12.0);
    }

    // Gaussian (default)
    return noise_model["stddev"] ? noise_model["stddev"].as<double>(fallback) : fallback;
}

}  // namespace sensor_config

#endif  // INT_SYS_FP_SENSOR_CONFIG_HPP
