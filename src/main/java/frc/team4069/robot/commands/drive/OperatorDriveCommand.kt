package frc.team4069.robot.commands.drive

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.saturn.lib.commands.SaturnCommand

class OperatorDriveCommand : SaturnCommand(Drivetrain) {
    override suspend fun execute() {
        // Limit the max output of the drivetrain if the elevator is extended to reduce chance of falling
        val shouldSlow = Elevator.position > (Elevator.MAX_HEIGHT / 2)
        val turn = OI.driveTurn
        val speed = if(shouldSlow) OI.driveSpeed * 0.5 else OI.driveSpeed

        if(!OI.onlyAux) {
            Drivetrain.curvatureDrive(speed, turn, speed == 0.0)
            // AUX is one gearbox, so can't apply differential algorithms
            if (OI.usingAux) {
                Drivetrain.setAux(speed)
            }else {
                Drivetrain.setAux(0.0)
            }
        }else {
            Drivetrain.setAux(speed)
        }
    }
}
