package frc.team4069.robot.commands.intake

import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

class OverdriveIntakeCommand : SaturnCommand() {
    override fun initialize() {
        Intake.setIntakeOverdrive(true)
        Intake.setPivotDutyCycle(0.2)
    }

    override fun end(_interrupted: Boolean) {
        Intake.setIntakeOverdrive(false)
        Intake.setPivotDutyCycle(0.0)
    }
}
