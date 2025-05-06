#include <urdf/model.h>
#include <Eigen/Core>

class RobotModel {
public:
  bool init(const std::string& urdf_str) {
    return model_.initString(urdf_str);
  }

  Eigen::Vector3d get_leg_dimensions() {
    Eigen::Vector3d lengths;
    return lengths;
  }

  std::map<std::string, double> get_link_masses() {
    std::map<std::string, double> masses;
    for (auto& [name, link] : model_.links_) {
      if (link->inertial) masses[name] = link->inertial->mass;
    }
    return masses;
  }

private:
  urdf::Model model_;
};