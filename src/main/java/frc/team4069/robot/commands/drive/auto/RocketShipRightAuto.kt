package frc.team4069.robot.commands.drive.auto

//fun RocketShipRightAuto() = sequential {
//    +parallel {
//
//        +FollowPathCommand(Trajectories.rocketRightOffset, zeroPose = true).markers {
//            Rectangle2d(
//                    Translation2d(11.8.feet, 1.feet),
//                    Translation2d(18.feet, 5.feet)
//            ) += {
//                AlignHatchIntakeCommand().start()
//                Elevator.setPosition(Elevator.Position.LOW_ROCKET_CARGO_HATCH.length)
//                Intake.extended = true
//            }
//        }
//
//        +sequential {
//            +DelayCommand(1.second)
//            +InstantRunnableCommand { SlideIntake.setPosition(11.inch) }
//        }
//    }
//    +SetElevatorPositionCommand(0.inch)
//    +FollowPathCommand(Trajectories.rocketRightToHalfHP)
//    +FollowPathCommand(Trajectories.rightHalfToHP).markers {
//        Rectangle2d(
//                Translation2d(0.feet, 0.feet),
//                Translation2d(7.feet, 7.feet)
//        ) += {
//            Elevator.setPosition(Elevator.Position.PORTAL_INTAKE.length)
//        }
//    }
//}
//