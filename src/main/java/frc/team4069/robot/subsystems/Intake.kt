package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.intake.AlignHatchIntakeCommand
import frc.team4069.robot.commands.intake.OperatorControlIntakeCommand
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.Rotation2d
import frc.team4069.saturn.lib.mathematics.units.degree
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import kotlin.math.absoluteValue
import kotlin.properties.Delegates

object Intake : SaturnSubsystem() {
    private val talon = TalonSRX(RobotMap.Intake.MAIN_SRX)

    val pivot = TalonSRX(RobotMap.Intake.PIVOT_SRX)
    val slide = SaturnSRX(RobotMap.Intake.SLIDE_SRX, Constants.INTAKE_SLIDE_MODEL)

    var overdrive = false

    const val PIVOT_STEP = 50

    var upperTarget = 0.0
    var lowerTarget = 2200.0

    var pivotState by Delegates.observable(PivotPosition.RETRACTED) { _, _, new ->
        when (new) {
            PivotPosition.EXTENDED -> {
                pivot.selectProfileSlot(1, 0)
            }
            PivotPosition.RETRACTED -> {
                pivot.selectProfileSlot(0, 0)
            }
        }
    }

    init {
        slide.apply {
            val maxVel = Constants.INTAKE_SLIDE_MODEL.toNativeUnitVelocity(38.65.inch.velocity)
            talon.config_kP(0, 1.2)
            talon.config_kF(0, 1023 / maxVel.STUPer100ms)
            motionProfileAcceleration = 69.inch.acceleration
            motionProfileCruiseVelocity = 55.inch.velocity

            useMotionProfileForPosition = true

            talon.configForwardSoftLimitThreshold(Constants.INTAKE_SLIDE_MODEL.toNativeUnitPosition(11.inch).value.toInt())
            talon.configForwardSoftLimitEnable(true)
            talon.configReverseSoftLimitThreshold(0)
            talon.configReverseSoftLimitEnable(true)

            outputInverted = true
        }

        pivot.apply {
            setSensorPhase(true)
            config_kP(0, 0.4)
            config_kD(0, 0.1)
//            kI = 0.0001
            config_kI(0, 0.00005)

            config_kP(1, 0.5)
            config_kD(1, 1.3)
            configClosedloopRamp(0.95)
        }
    }

    val angle: Rotation2d
        get() = (-(360.0 * (pivot.selectedSensorPosition / 7540)).degree + 90.degree)


    override fun lateInit() {
        pivot.selectedSensorPosition = 0
        slide.encoder.resetPosition(0.inch)
    }

    override fun periodic() {
        SmartDashboard.putNumber("Slide Position", slidePosition.inch)
        if (!overdrive) {
            when (pivotState) {
                PivotPosition.RETRACTED -> {
                    pivot.set(ControlMode.Position, upperTarget, DemandType.ArbitraryFeedForward, -0.05 * angle.cos)
                }
                PivotPosition.EXTENDED -> {
                    if (angle.degree.absoluteValue < 35.0) {
                        pivot.set(ControlMode.Position, lowerTarget)
                    } else {
                        pivot.set(ControlMode.Position, lowerTarget, DemandType.ArbitraryFeedForward, -0.3)
                    }
                }
            }
        }
    }

    fun set(spd: Double) {
        talon.set(ControlMode.PercentOutput, spd)
    }

    var slidePosition: Length
        get() = slide.encoder.position
        set(value) {
            slide.setPosition(value)
        }


    fun get() = talon.motorOutputPercent

    override fun teleopReset() {
        OperatorControlIntakeCommand().start()
//        ControlSlideCommand().start()
        AlignHatchIntakeCommand().start()
    }

    override fun autoReset() {
//        AlignHatchIntakeCommand().start()
    }

    fun setSlide(speed: Double) {
        slide.setDutyCycle(speed)
    }

    enum class PivotPosition {
        EXTENDED,
        RETRACTED
    }
}
