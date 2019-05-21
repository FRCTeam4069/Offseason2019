package frc.team4069.robot.commands.drive.auto

import frc.team4069.robot.Trajectories
import frc.team4069.robot.commands.drive.DriveStraightCommand
import frc.team4069.robot.commands.drive.DriveStraightToTape
import frc.team4069.robot.commands.drive.FollowPathCommand
import frc.team4069.robot.commands.elevator.SetElevatorPositionCommand
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.DelayCommand
import frc.team4069.saturn.lib.commands.InstantRunnableCommand
import frc.team4069.saturn.lib.commands.sequential
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Rectangle2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Translation2d
import frc.team4069.saturn.lib.mathematics.units.feet
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.second

@Suppress("FunctionName")
fun CargoShipRight() = sequential {
    +FollowPathCommand(Trajectories.cargoShipRightOffset, zeroPose = true).markers {
        Rectangle2d(
                Translation2d(10.feet, 9.feet),
                Translation2d(12.feet, 12.feet)
        ) += {
            Intake.slidePosition = 11.inch
            Elevator.set(6.inch)
        }
    }
    +SetElevatorPositionCommand(Elevator.Position.CARGO_SHIP_HATCH)
    +DelayCommand(0.75.second)
    +InstantRunnableCommand { Intake.pivotState = Intake.PivotPosition.EXTENDED }
    +DelayCommand(0.5.second)
    +DriveStraightToTape()
    +SetElevatorPositionCommand(Elevator.Position.MINIMUM)
    +DriveStraightCommand(-1.feet)
    +DelayCommand(0.5.second)
//    +FollowPathCommand(Trajectories.cargoShipHPHalf).markers {
//        Rectangle2d(
//                Translation2d(),
//                Translation2d()
//        ) += {
//            Intake.pivotState = Intake.PivotPosition.RETRACTED
//        }
//    }
//
//    +FollowPathCommand(Trajectories.rightHalfToHP).markers {
//        Rectangle2d(
//                Translation2d(8.feet, 2.feet),
//                Translation2d(10.feet, 4.feet)
//        ) += {
//            Elevator.set(Elevator.Position.PORTAL_INTAKE.length)
//            Intake.pivotState = Intake.PivotPosition.EXTENDED
//        }
//    }
//    +DriveStraightToTape()

}
