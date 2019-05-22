#include "subsystems/ElevatorCoeffs.hpp"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<2, 1, 1> MakeElevatorPlantCoeffs() {
  Eigen::Matrix<double, 2, 2> A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.0038123917094548044;
  A(1, 0) = 0.0;
  A(1, 1) = 0.09251846318419062;
  Eigen::Matrix<double, 2, 1> B;
  B(0, 0) = 0.000587130443210727;
  B(1, 0) = 0.08610920599650167;
  Eigen::Matrix<double, 1, 2> C;
  C(0, 0) = 1.0;
  C(0, 1) = 0.0;
  Eigen::Matrix<double, 1, 1> D;
  D(0, 0) = 0.0;
  return frc::StateSpacePlantCoeffs<2, 1, 1>(A, B, C, D);
}

frc::StateSpaceControllerCoeffs<2, 1, 1> MakeElevatorControllerCoeffs() {
  Eigen::Matrix<double, 1, 2> K;
  K(0, 0) = 197.74500975121012;
  K(0, 1) = 1.5868649265235129;
  Eigen::Matrix<double, 1, 2> Kff;
  Kff(0, 0) = 27.107309014237973;
  Kff(0, 1) = 9.9389704745734;
  Eigen::Matrix<double, 1, 1> Umin;
  Umin(0, 0) = -12.0;
  Eigen::Matrix<double, 1, 1> Umax;
  Umax(0, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<2, 1, 1>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<2, 1, 1> MakeElevatorObserverCoeffs() {
  Eigen::Matrix<double, 2, 1> K;
  K(0, 0) = 0.9999757217645118;
  K(1, 0) = 0.6994463408695476;
  return frc::StateSpaceObserverCoeffs<2, 1, 1>(K);
}

frc::StateSpaceLoop<2, 1, 1> MakeElevatorLoop() {
  return frc::StateSpaceLoop<2, 1, 1>(MakeElevatorPlantCoeffs(),
                                      MakeElevatorControllerCoeffs(),
                                      MakeElevatorObserverCoeffs());
}
