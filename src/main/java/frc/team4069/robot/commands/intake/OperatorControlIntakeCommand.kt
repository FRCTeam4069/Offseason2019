package frc.team4069.robot.commands.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

class OperatorControlIntakeCommand : SaturnCommand(Intake) {
    override suspend fun execute() {
        val speed = OI.intakeSpeed
        Intake.set(speed)
    }
}