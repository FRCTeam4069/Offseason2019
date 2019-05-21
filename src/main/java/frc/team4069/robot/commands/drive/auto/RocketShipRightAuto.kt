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
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Rectangle2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Translation2d
import frc.team4069.saturn.lib.mathematics.units.feet
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.second

fun RocketShipRightAuto() = sequential {
    +parallel {

        +FollowPathCommand(Trajectories.rocketRightOffset, zeroPose = true).markers {
            Rectangle2d(
                    Translation2d(11.8.feet, 1.feet),
                    Translation2d(18.feet, 5.feet)
            ) += {
                AlignHatchIntakeCommand().start()
                Elevator.set(Elevator.Position.LOW_ROCKET_CARGO_HATCH.length)
                Intake.pivotState = Intake.PivotPosition.EXTENDED
            }
        }

        +sequential {
            +DelayCommand(1.second)
            +InstantRunnableCommand { Intake.slidePosition = 11.inch }
        }
    }
    +SetElevatorPositionCommand(0.inch)
    +FollowPathCommand(Trajectories.rocketRightToHalfHP)
    +FollowPathCommand(Trajectories.rightHalfToHP).markers {
        Rectangle2d(
                Translation2d(0.feet, 0.feet),
                Translation2d(7.feet, 7.feet)
        ) += {
            Elevator.set(Elevator.Position.PORTAL_INTAKE.length)
        }
    }
}
