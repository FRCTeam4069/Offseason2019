#pragma once

#include <frc/controller/StateSpaceControllerCoeffs.h>
#include <frc/controller/StateSpaceLoop.h>
#include <frc/controller/StateSpaceObserverCoeffs.h>
#include <frc/controller/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<2, 1, 2> MakeElevatorPlantCoeffs();
frc::StateSpaceControllerCoeffs<2, 1, 2> MakeElevatorControllerCoeffs();
frc::StateSpaceObserverCoeffs<2, 1, 2> MakeElevatorObserverCoeffs();
frc::StateSpaceLoop<2, 1, 2> MakeElevatorLoop();
