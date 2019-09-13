package frc.team4069.robot.commands.drive.auto

//fun RocketShipLeftAuto() = sequential {
//    +parallel {
//        +FollowPathCommand(Trajectories.rocketLeftOffset, zeroPose = true).markers {
//            Rectangle2d(
//                     27 - 1 = 26
//                     27 - 5 = 22
//                    Translation2d(11.8.feet, 22.feet),
//                    Translation2d(18.feet, 26.feet)
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
//    +FollowPathCommand(Trajectories.rocketLeftToHalfHP)
//    +FollowPathCommand(Trajectories.leftHalfToHP).markers {
//        Rectangle2d(
//                Translation2d(0.feet, 20.feet),
//                Translation2d(7.feet, 27.feet)
//        ) += {
//            Elevator.setPosition(Elevator.Position.PORTAL_INTAKE.length)
//        }
//    }
//}
