package frc.team4069.robot.commands.drive.auto

import frc.team4069.robot.Trajectories
import frc.team4069.robot.commands.drive.FollowPathCommand
import frc.team4069.robot.commands.elevator.SetElevatorPositionCommand
import frc.team4069.robot.commands.intake.AlignHatchIntakeCommand
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.robot.subsystems.intake.SlideIntake
import frc.team4069.saturn.lib.commands.DelayCommand
import frc.team4069.saturn.lib.commands.InstantRunnableCommand
import frc.team4069.saturn.lib.commands.parallel
import frc.team4069.saturn.lib.commands.sequential
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Rectangle2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Translation2d
import frc.team4069.saturn.lib.mathematics.units.feet
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.second

fun RocketShipLeftAuto() = sequential {
    +parallel {
        +FollowPathCommand(Trajectories.rocketLeftOffset, zeroPose = true).markers {
            Rectangle2d(
                    // 27 - 1 = 26
                    // 27 - 5 = 22
                    Translation2d(11.8.feet, 22.feet),
                    Translation2d(18.feet, 26.feet)
            ) += {
                AlignHatchIntakeCommand().start()
                Elevator.setPosition(Elevator.Position.LOW_ROCKET_CARGO_HATCH.length)
                Intake.extended = true
            }
        }

        +sequential {
            +DelayCommand(1.second)
            +InstantRunnableCommand { SlideIntake.setPosition(11.inch) }
        }
    }
    +SetElevatorPositionCommand(0.inch)
    +FollowPathCommand(Trajectories.rocketLeftToHalfHP)
    +FollowPathCommand(Trajectories.leftHalfToHP).markers {
        Rectangle2d(
                Translation2d(0.feet, 20.feet),
                Translation2d(7.feet, 27.feet)
        ) += {
            Elevator.setPosition(Elevator.Position.PORTAL_INTAKE.length)
        }
    }
}
