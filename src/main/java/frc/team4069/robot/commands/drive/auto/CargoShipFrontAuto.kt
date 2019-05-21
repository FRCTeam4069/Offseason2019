package frc.team4069.robot.commands.drive.auto

import frc.team4069.robot.Trajectories
import frc.team4069.robot.commands.drive.DriveStraightToTape
import frc.team4069.robot.commands.drive.FollowPathCommand
import frc.team4069.robot.commands.elevator.SetElevatorPositionCommand
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.DelayCommand
import frc.team4069.saturn.lib.commands.InstantRunnableCommand
import frc.team4069.saturn.lib.commands.parallel
import frc.team4069.saturn.lib.commands.sequential
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.second

@Suppress("FunctionName")
fun CargoShipFrontAuto() = sequential {
    +FollowPathCommand(Trajectories.frontCargoShip, zeroPose = true)
    +parallel {
        +InstantRunnableCommand { Intake.slidePosition = 11.inch }
        +SetElevatorPositionCommand(6.inch)
    }
    +DelayCommand(0.75.second)
    +InstantRunnableCommand { Intake.pivotState = Intake.PivotPosition.EXTENDED }
    +DelayCommand(0.5.second)
    +SetElevatorPositionCommand(Elevator.Position.CARGO_SHIP_HATCH)
    +DriveStraightToTape()
    //TODO: Get second hatch
}