package frc.team4069.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.intake.OperatorControlIntakeCommand
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.Radian
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.conversions.degree
import frc.team4069.saturn.lib.mathematics.units.conversions.radian
import frc.team4069.saturn.lib.mathematics.units.degree
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import kotlin.math.absoluteValue
import kotlin.math.cos

object Intake : SaturnSubsystem(), Loggable {
    private val talon = TalonSRX(RobotMap.Intake.MAIN_SRX)

    val pivot = TalonSRX(RobotMap.Intake.PIVOT_SRX)

    private val periodicIO = PeriodicIO()

    const val PIVOT_STEP = 50

    var upperTarget = 0.0
    var lowerTarget = 2200.0

    var extended: Boolean
        get() = periodicIO.pivotExtended
        set(value) {
            if(value) {
                pivot.selectProfileSlot(1, 0)
            } else {
                pivot.selectProfileSlot(0, 0)
            }

            periodicIO.pivotExtended = value
        }

    fun setIntakeOverdrive(overdrive: Boolean) {
        periodicIO.intakeOverdrive = overdrive
    }

    init {
        pivot.apply {
            setSensorPhase(true)
            config_kP(0, 0.4)
            config_kD(0, 0.1)
            config_kI(0, 0.00005)

            config_kP(1, 0.5)
            config_kD(1, 1.3)
            configClosedloopRamp(0.95)
        }
    }

    val angle: SIUnit<Radian>
        get() = (-(360.0 * (pivot.selectedSensorPosition / 7540)).degree + 90.degree)


    override fun lateInit() {
        pivot.selectedSensorPosition = 0
    }

    override fun periodic() {
        periodicIO.intakeVoltage = talon.motorOutputVoltage
        periodicIO.intakeCurrent = talon.outputCurrent
        periodicIO.pivotVoltage = pivot.motorOutputVoltage
        periodicIO.pivotCurrent = pivot.outputCurrent
        periodicIO.pivotAngle = angle.degree

        if (!periodicIO.intakeOverdrive) {
            if (periodicIO.pivotExtended) {
                if (angle.degree.absoluteValue < 35.0) {
                    pivot.set(ControlMode.Position, lowerTarget)
                } else {
                    pivot.set(ControlMode.Position, lowerTarget, DemandType.ArbitraryFeedForward, -0.3)
                }
            } else {
                pivot.set(ControlMode.Position, upperTarget, DemandType.ArbitraryFeedForward, -0.05 * cos(angle.radian))
            }
        } else {
            pivot.set(ControlMode.PercentOutput, periodicIO.pivotSpeed)
        }

        talon.set(ControlMode.PercentOutput, periodicIO.intakeSpeed)
    }

    fun setDutyCycle(spd: Double) {
        periodicIO.intakeSpeed = spd
    }

    fun setPivotDutyCycle(value: Double) {
        periodicIO.pivotSpeed = value
    }

    fun get() = talon.motorOutputPercent

    override fun teleopReset() {
        OperatorControlIntakeCommand().schedule()
    }

    private class PeriodicIO : Loggable {
        override fun configureLayoutType() = BuiltInLayouts.kGrid
        override fun skipLayout() = true

        @Log.VoltageView(name = "Intake Voltage", rowIndex = 0, columnIndex = 0)
        var intakeVoltage: Double = 0.0

        @Log(name = "Intake Current", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
        var intakeCurrent: Double = 0.0

        @Log.VoltageView(name = "Pivot Voltage", rowIndex = 0, columnIndex = 2)
        var pivotVoltage: Double = 0.0

        @Log(name = "Pivot Current", width = 2, height = 1, rowIndex = 1, columnIndex = 2)
        var pivotCurrent: Double = 0.0

        @Log(name = "Pivot Angle", width = 2, height = 1, rowIndex = 2, columnIndex = 1)
        var pivotAngle: Double = 0.0

        // Outputs
        var intakeSpeed: Double = 0.0
        var pivotSpeed: Double = 0.0
        var pivotExtended: Boolean = false
        var intakeOverdrive: Boolean = false
    }
}
