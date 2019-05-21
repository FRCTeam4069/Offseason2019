package frc.team4069.robot.commands.intake

import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.commands.SaturnCommand

// This command is meant to be cancelled by the button scheduler
// For equivalent behaviour in auto just use InstantRunnableCommands
class DropIntakeCommand : SaturnCommand() {
    override suspend fun initialize() {
        Intake.pivotState = Intake.PivotPosition.EXTENDED
    }

    override suspend fun dispose() {
        println("Calling dispose. Should retract")
        Intake.pivotState = Intake.PivotPosition.RETRACTED
    }
}