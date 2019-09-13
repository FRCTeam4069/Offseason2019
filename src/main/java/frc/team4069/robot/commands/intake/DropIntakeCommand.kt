package frc.team4069.robot.commands.intake

import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

// This command is meant to be cancelled by the button scheduler
// For equivalent behaviour in auto just use InstantRunnableCommands
class DropIntakeCommand : SaturnCommand() {
    override fun initialize() {
        Intake.extended = true
    }

    override fun end(_interrupted: Boolean) {
        Intake.extended = false
    }
}