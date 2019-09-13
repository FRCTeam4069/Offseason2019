package frc.team4069.robot.commands.intake

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

class OperatorControlIntakeCommand : SaturnCommand(Intake) {
    override fun execute() {
        val speed = OI.intakeSpeed
        Intake.setDutyCycle(speed)
    }
}