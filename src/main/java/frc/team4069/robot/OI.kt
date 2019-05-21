package frc.team4069.robot

import edu.wpi.first.wpilibj.GenericHID
import frc.team4069.robot.commands.elevator.SetElevatorPositionCommand
import frc.team4069.robot.commands.intake.OverdriveIntakeCommand
import frc.team4069.robot.subsystems.BoostCaboose
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.Intake
import frc.team4069.saturn.lib.hid.*
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.util.deadband

object OI {
    // Whether the AUX drivebase is being used together with the primary one
    var usingAux = false
    // Whether the AUX drivebase is being used on its own
    var onlyAux = false

    var sliderOperatorControl = false

    // Controllers and command wiring goes here
    val driveController = xboxController(0) {
        button(kStart) {
            changeOn {
                usingAux = !usingAux
            }
        }
        button(kBack) {
            changeOn {
                onlyAux = !onlyAux
            }
        }

        button(kX) {
            changeOn {
                sliderOperatorControl = !sliderOperatorControl
            }
        }
    }

    val operatorController = xboxController(1) {
        button(kBumperRight) {
//            change(DropIntakeCommand())
            changeOn {
                when(Intake.pivotState) {
                    Intake.PivotPosition.EXTENDED -> Intake.pivotState = Intake.PivotPosition.RETRACTED
                    Intake.PivotPosition.RETRACTED -> Intake.pivotState = Intake.PivotPosition.EXTENDED
                }
            }
        }

        button(kBumperLeft) {
            change(OverdriveIntakeCommand())
        }

        button(kB) {
            changeOn {
                BoostCaboose.toggle()
            }
        }

        button(kA) {
            changeOn {
                Intake.lowerTarget -= Intake.PIVOT_STEP
            }
        }

        button(kY) {
            changeOn {
                Intake.lowerTarget += Intake.PIVOT_STEP
            }
        }

        button(kStart) {
            changeOn {
                Elevator.position = 0.inch
            }
        }

        pov(POVSide.DOWN).changeOn(SetElevatorPositionCommand(Elevator.Position.MINIMUM))
        pov(POVSide.LEFT).changeOn(SetElevatorPositionCommand(Elevator.Position.LOW_ROCKET_CARGO_HATCH))
        pov(POVSide.RIGHT).changeOn(SetElevatorPositionCommand(Elevator.Position.MID_ROCKET_CARGO_HATCH))
        pov(POVSide.UP).changeOn(SetElevatorPositionCommand(Elevator.Position.HIGH_ROCKET_CARGO_HATCH))
    }

    val driveSpeed: Double
        get() {
            val out = driveController.getTriggerAxis(GenericHID.Hand.kRight) - driveController.getTriggerAxis(GenericHID.Hand.kLeft)
            return out.deadband(0.3)
        }

    val driveTurn: Double
        get() {
            val out = driveController.getX(GenericHID.Hand.kLeft).deadband(0.2)
            return out * 0.75
        }

    val elevatorSpeed: Double
        get() {
            return -operatorController.getY(GenericHID.Hand.kRight).deadband(0.2)
        }

    val climberSpeed: Double
        get() {
            return operatorController.getY(GenericHID.Hand.kLeft).deadband(0.2)
        }

    val intakeSpeed: Double
        get() {
            val fwd = operatorController.getTriggerAxis(GenericHID.Hand.kRight)
            val back = operatorController.getTriggerAxis(GenericHID.Hand.kLeft)
            return (fwd - back).deadband(0.1).coerceIn(-1.0..0.65)
        }

    val slideSpeed: Double
        get() = driveController.getX(GenericHID.Hand.kRight).deadband(0.2)
}