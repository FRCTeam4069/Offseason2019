package frc.team4069.robot.commands.drive.auto

import frc.team4069.robot.commands.drive.OperatorDriveCommand
import frc.team4069.robot.commands.elevator.OperatorElevatorCommand
import frc.team4069.robot.commands.intake.AlignHatchIntakeCommand
import frc.team4069.robot.commands.intake.OperatorControlIntakeCommand
import frc.team4069.saturn.lib.commands.parallel

@Suppress("FunctionName")
fun TeleoperatedSandstorm() = parallel {
    +OperatorDriveCommand()
    +AlignHatchIntakeCommand()
    +OperatorElevatorCommand()
    +OperatorControlIntakeCommand()
}