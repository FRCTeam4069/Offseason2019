package frc.team4069.robot.commands.intake

import frc.team4069.robot.commands.drive.DriveStraightCommand
import frc.team4069.robot.commands.drive.NavigateToTapeWithoutLidar
import frc.team4069.robot.commands.elevator.SetElevatorPositionCommand
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.DelayCommand
import frc.team4069.saturn.lib.commands.InstantRunnableCommand
import frc.team4069.saturn.lib.commands.sequential
import frc.team4069.saturn.lib.mathematics.units.*
import kotlinx.coroutines.ObsoleteCoroutinesApi

// Command to be started in teleop to grab the hatch panel from the HP station once roughly aligned
@ObsoleteCoroutinesApi
fun loadAtHPCommand(end: Rotation2d = 180.degree) =
        sequential {
            +InstantRunnableCommand { Intake.pivotState = Intake.PivotPosition.EXTENDED }
            +SetElevatorPositionCommand(Elevator.Position.PORTAL_INTAKE)
            +DelayCommand(1.second)
            +NavigateToTapeWithoutLidar(endRot = end)
            +DelayCommand(0.5.second)
            +InstantRunnableCommand { Intake.pivotState = Intake.PivotPosition.RETRACTED }
            +SetElevatorPositionCommand(Elevator.position + 3.inch)
            +InstantRunnableCommand { Elevator.set(0.6) }
            +DelayCommand(0.4.second)
            +DriveStraightCommand(-1.feet)
            +InstantRunnableCommand { Elevator.set(0.0) }
            +DelayCommand(0.2.second)
            +SetElevatorPositionCommand(Elevator.Position.MINIMUM)
        }
