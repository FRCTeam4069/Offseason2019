package frc.team4069.robot.commands.drive.auto

//@Suppress("FunctionName")
//fun CargoShipFrontAuto() = sequential {
//    +FollowPathCommand(Trajectories.frontCargoShip, zeroPose = true)
//    +parallel {
//        +InstantRunnableCommand { SlideIntake.setPosition(11.inch) }
//        +SetElevatorPositionCommand(6.inch)
//    }
//    +DelayCommand(0.75.second)
//    +InstantRunnableCommand { Intake.extended = true }
//    +DelayCommand(0.5.second)
//    +SetElevatorPositionCommand(Elevator.Position.CARGO_SHIP_HATCH)
//    +DriveStraightToTape()
//    TODO: Get second hatch
//}