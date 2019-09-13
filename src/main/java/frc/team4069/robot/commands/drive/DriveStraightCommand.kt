package frc.team4069.robot.commands.drive

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.saturn.lib.mathematics.units.Meter
import frc.team4069.saturn.lib.mathematics.units.SIUnit

/**
 * Drives straight to a distance relative to the current position of the drive base
 */
class DriveStraightCommand(val length: SIUnit<Meter>) : InstantCommand() {
    override fun initialize() {
        Drivetrain.motionMagicRelative(length)
    }
}
