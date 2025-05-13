#include "unitree_controller/pd_controller.hpp"

namespace unitree_controller
{

PDController::PDController(const Vector12d& qJ, const Vector12d& dqJ, const Vector12d& tauJ, 
                           const Vector12d& Kp, const Vector12d& Kd)
  : qJ_cmd_(qJ),  
    dqJ_cmd_(dqJ),  
    tauJ_cmd_(tauJ),  
    Kp_cmd_(Kp),  
    Kd_cmd_(Kd) {
  if (Kp.minCoeff() < 0.0) {
    throw std::invalid_argument("[PDController] Kp.minCoeff() must be non-negative");
  }
  if (Kd.minCoeff() < 0.0) {
    throw std::invalid_argument("[PDController] Kp.minCoeff() must be non-negative");
  }
}


PDController PDController::ZeroTorqueController() 
{
  const Vector12d qJ = Vector12d::Constant(UNITREE_LEGGED_SDK::PosStopF);
  const Vector12d dqJ = Vector12d::Constant(UNITREE_LEGGED_SDK::VelStopF);
  const Vector12d tauJ = Vector12d::Zero();
  const Vector12d Kp = Vector12d::Zero();
  const Vector12d Kd = Vector12d::Zero();
  return PDController(std::move(qJ), std::move(dqJ), std::move(tauJ), std::move(Kp), std::move(Kd));
}


PDController PDController::StandingUpController() 
{
  Vector12d qJ;
  qJ << -0.05, 0.67, -1.3, 
        0.05, 0.67, -1.3, 
        -0.05, 0.67, -1.3,
        0.05, 0.67, -1.3;
  const Vector12d dqJ = Vector12d::Zero();
  const Vector12d tauJ = Vector12d::Zero();
  const Vector12d Kp = Vector12d::Constant(50.0); // original value is 20 
  const Vector12d Kd = Vector12d::Constant(15.0); // original value is 10 
  return PDController(std::move(qJ), std::move(dqJ), std::move(tauJ), std::move(Kp), std::move(Kd));
}


PDController PDController::SittingDownController() 
{
  Vector12d qJ;
  // qJ << -0.19, 1.0, -2.5, 
  //       0.19, 1.0, -2.5, 
  //       -0.19, 0.8, -2.3, 
  //       0.19, 0.8, -2.3;
  qJ << 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;
  const Vector12d dqJ = Vector12d::Zero();
  const Vector12d tauJ = Vector12d::Zero();
  const Vector12d Kp = Vector12d::Constant(90.0); // original value is 10 
  const Vector12d Kd = Vector12d::Constant(16.0); // original value is 15 
  return PDController(std::move(qJ), std::move(dqJ), std::move(tauJ), std::move(Kp), std::move(Kd));
}

} // namespace unitree_controller