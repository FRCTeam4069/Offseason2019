package frc.team4069.robot.commands.drive

import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.commands.InstantCommand
import frc.team4069.saturn.lib.mathematics.units.Length

/**
 * Drives straight to a distance relative to the current position of the drive base
 */
class DriveStraightCommand(val length: Length) : InstantCommand() {
    override suspend fun initialize() {
        Drivetrain.motionMagicRelative(length)
    }
}
