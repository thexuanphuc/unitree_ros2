// find_package(yaml-cpp REQUIRED)
// add_executable(my_app main.cpp)
// target_link_libraries(my_app PRIVATE yaml-cpp)

#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

struct Config {
    std::string urdf_file;
    double standing_height;
    double step_length;
    double swing_height;
    std::string trajectory_type;
    int N;
    double dt;
    double dq_max;
    bool fix_hip;
    std::vector<double> q0;
    double w_pos;
    double w_dq;
    double w_smooth;
    double w_y;
    struct {
        std::vector<double> q_min;
        std::vector<double> q_max;
    } joint_limits;
    double com_shifting;
};

Config loadConfig(const std::string& path) {
    YAML::Node config = YAML::LoadFile(path);
    
    Config cfg;
    cfg.standing_height = config["standing_height"].as<double>();
    cfg.step_length = config["step_length"].as<double>();
    cfg.swing_height = config["swing_height"].as<double>();
    // cfg.trajectory_type = config["trajectory_type"].as<std::string>();
    cfg.N = config["N"].as<int>();
    cfg.dt = config["dt"].as<double>();
    cfg.dq_max = config["dq_max"].as<double>();
    cfg.fix_hip = config["fix_hip"].as<bool>();
    cfg.q0 = config["q0"].as<std::vector<double>>();
    cfg.w_pos = config["w_pos"].as<double>();
    cfg.w_dq = config["w_dq"].as<double>();
    cfg.w_smooth = config["w_smooth"].as<double>();
    cfg.w_y = config["w_y"].as<double>();
    
    cfg.joint_limits.q_min = config["joint_limits"]["q_min"].as<std::vector<double>>();
    cfg.joint_limits.q_max = config["joint_limits"]["q_max"].as<std::vector<double>>();
    
    cfg.com_shifting = config["com_shifting"].as<double>();
    
    return cfg;
}

// Usage
int main() {
    Config cfg = loadConfig("robot_description.yaml");
    return 0;
}
