package frc.team4069.robot.commands.drive.auto

import frc.team4069.robot.Trajectories
import frc.team4069.robot.commands.drive.FollowPathCommand
import frc.team4069.robot.commands.elevator.SetElevatorPositionCommand
import frc.team4069.robot.commands.intake.AlignHatchIntakeCommand
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.DelayCommand
import frc.team4069.saturn.lib.commands.InstantRunnableCommand
import frc.team4069.saturn.lib.commands.parallel
import frc.team4069.saturn.lib.commands.sequential
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.second

fun RocketShipRightBackwardsAuto() = sequential {
    +parallel {
        +FollowPathCommand(Trajectories.backRightRocket, zeroPose = true)
        +sequential {
            +DelayCommand(2.second)
            +InstantRunnableCommand { Intake.slidePosition = 11.inch }
        }
    }
    +parallel {
        +parallel {
            +SetElevatorPositionCommand(Elevator.Position.LOW_ROCKET_CARGO_HATCH, instant = true)
            +InstantRunnableCommand { Intake.pivotState = Intake.PivotPosition.EXTENDED }
            +InstantRunnableCommand { AlignHatchIntakeCommand().start() }
        }
        +FollowPathCommand(Trajectories.backRocketToTarget)
    }
    +SetElevatorPositionCommand(Elevator.Position.MINIMUM)
    +FollowPathCommand(Trajectories.reverseBackRocketToTarget)
    +FollowPathCommand(Trajectories.backMarkToHP)
}

