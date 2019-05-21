package frc.team4069.robot.commands.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

class OverdriveIntakeCommand : SaturnCommand() {
    override suspend fun initialize() {
        Intake.overdrive = true
        Intake.pivot.set(ControlMode.PercentOutput, 0.2)
    }

    override suspend fun dispose() {
        Intake.overdrive = false
        Intake.pivot.neutralOutput()
    }
}
