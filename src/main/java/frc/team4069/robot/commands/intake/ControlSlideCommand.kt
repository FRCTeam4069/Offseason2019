package frc.team4069.robot.commands.intake

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

class ControlSlideCommand : SaturnCommand() {
    override suspend fun execute() {
        Intake.setSlide(OI.slideSpeed)
    }
}
