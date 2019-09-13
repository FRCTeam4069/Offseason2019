package frc.team4069.robot.commands.drive.auto


//fun RocketShipRightBackwardsAuto() = sequential {
//    +parallel {
//        +FollowPathCommand(Trajectories.backRightRocket, zeroPose = true)
//        +sequential {
//            +DelayCommand(2.second)
//            +InstantRunnableCommand { SlideIntake.setPosition(11.inch) }
//        }
//    }
//    +parallel {
//        +parallel {
//            +SetElevatorPositionCommand(Elevator.Position.LOW_ROCKET_CARGO_HATCH, instant = true)
//            +InstantRunnableCommand { Intake.extended = true }
//            +InstantRunnableCommand { AlignHatchIntakeCommand().start() }
//        }
//        +FollowPathCommand(Trajectories.backRocketToTarget)
//    }
//    +SetElevatorPositionCommand(Elevator.Position.MINIMUM)
//    +FollowPathCommand(Trajectories.reverseBackRocketToTarget)
//    +FollowPathCommand(Trajectories.backMarkToHP)
//}

