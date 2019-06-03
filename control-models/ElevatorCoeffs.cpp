#include "subsystems/ElevatorCoeffs.hpp"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<2, 1, 2> MakeElevatorPlantCoeffs() {
  Eigen::Matrix<double, 2, 2> A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.000672170658888835;
  A(1, 0) = 0.0;
  A(1, 1) = 3.458816828536931e-07;
  Eigen::Matrix<double, 2, 1> B;
  B(0, 0) = 0.0003540400308538644;
  B(1, 0) = 0.037955230038088185;
  Eigen::Matrix<double, 2, 2> C;
  C(0, 0) = 1.0;
  C(0, 1) = 0.0;
  C(1, 0) = 0.0;
  C(1, 1) = 1.0;
  Eigen::Matrix<double, 2, 1> D;
  D(0, 0) = 0.0;
  D(1, 0) = 0.0;
  return frc::StateSpacePlantCoeffs<2, 1, 2>(A, B, C, D);
}

frc::StateSpaceControllerCoeffs<2, 1, 2> MakeElevatorControllerCoeffs() {
  Eigen::Matrix<double, 1, 2> K;
  K(0, 0) = 367.5342218475731;
  K(0, 1) = 0.2470502277711982;
  Eigen::Matrix<double, 1, 2> Kff;
  Kff(0, 0) = 54.42900356510949;
  Kff(0, 1) = 14.58779495976241;
  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;
  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<2, 1, 2>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<2, 1, 2> MakeElevatorObserverCoeffs() {
  Eigen::Matrix<double, 2, 2> K;
  K(0, 0) = 0.3279215611246275;
  K(0, 1) = 6.247005370670148e-14;
  K(1, 0) = 6.247005370671573e-18;
  K(1, 1) = 0.9996001599360256;
  return frc::StateSpaceObserverCoeffs<2, 1, 2>(K);
}

frc::StateSpaceLoop<2, 1, 2> MakeElevatorLoop() {
  return frc::StateSpaceLoop<2, 1, 2>(MakeElevatorPlantCoeffs(),
                                      MakeElevatorControllerCoeffs(),
                                      MakeElevatorObserverCoeffs());
}
