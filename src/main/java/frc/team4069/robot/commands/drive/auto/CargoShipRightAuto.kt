package frc.team4069.robot.commands.drive.auto

//@Suppress("FunctionName")
//fun CargoShipRight() = sequential {
//    +FollowPathCommand(Trajectories.cargoShipRightOffset, zeroPose = true).markers {
//        Rectangle2d(
//                Translation2d(10.feet, 9.feet),
//                Translation2d(12.feet, 12.feet)
//        ) += {
//            SlideIntake.setPosition(11.inch)
//            Elevator.setPosition(6.inch)
//        }
//    }
//    +SetElevatorPositionCommand(Elevator.Position.CARGO_SHIP_HATCH)
//    +DelayCommand(0.75.second)
//    +InstantRunnableCommand { Intake.extended = true }
//    +DelayCommand(0.5.second)
//    +DriveStraightToTape()
//    +SetElevatorPositionCommand(Elevator.Position.MINIMUM)
//    +DriveStraightCommand(-1.feet)
//    +DelayCommand(0.5.second)
//}
//