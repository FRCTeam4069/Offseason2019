package frc.team4069.robot.commands.climber

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.BoostCaboose
import frc.team4069.saturn.lib.commands.SaturnCommand

class DriveClimberCommand : SaturnCommand(BoostCaboose) {
    override suspend fun initialize() {
        BoostCaboose.liftMaster.stopMotor()
    }

    override suspend fun execute() {
        println(OI.climberSpeed) // remove this and it breaks, no touchy
        BoostCaboose.set(OI.climberSpeed)
    }
}