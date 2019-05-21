package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
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
import frc.team4069.saturn.lib.mathematics.units.nativeunits.fromModel
import frc.team4069.saturn.lib.motor.NativeSaturnSRX
import frc.team4069.saturn.lib.motor.SaturnSRX
import kotlin.math.absoluteValue
import kotlin.properties.Delegates

object Intake : SaturnSubsystem() {
    private val talon = NativeSaturnSRX(RobotMap.Intake.MAIN_SRX, 4096.STU)

    val pivot = NativeSaturnSRX(RobotMap.Intake.PIVOT_SRX, 4096.STU)
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
            val maxVel = 38.65.inch.velocity.fromModel(Constants.INTAKE_SLIDE_MODEL)
            kP = 1.2
            kF = 1023 / maxVel.STUPer100ms
            motionAcceleration = 69.inch.acceleration
            motionCruiseVelocity = 55.inch.velocity

            softLimitForward = 11.inch
            softLimitForwardEnabled = true
            softLimitReverse = 0.inch
            softLimitReverseEnabled = true

            inverted = true
        }

        pivot.apply {
            setSensorPhase(true)
            kP = 0.4
            kD = 0.1
//            kI = 0.0001
            kI = 0.00005
            config_kP(1, 0.5)
            config_kD(1, 1.3)
            configClosedloopRamp(0.95)
//            softLimitReverse = 0.STU
//            softLimitReverseEnabled = true
        }
    }

    val angle: Rotation2d
        get() = (-(360.0 * (pivot.sensorPosition / 7540.STU)).degree + 90.degree)


    override fun lateInit() {
        pivot.sensorPosition = 0.STU
        slide.sensorPosition = 0.inch
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
        get() = slide.sensorPosition
        set(value) {
            slide.set(ControlMode.MotionMagic, value)
        }


    fun get() = talon.get()

    override fun teleopReset() {
        OperatorControlIntakeCommand().start()
//        ControlSlideCommand().start()
        AlignHatchIntakeCommand().start()
    }

    override fun autoReset() {
//        AlignHatchIntakeCommand().start()
    }

    fun setSlide(speed: Double) {
        slide.set(ControlMode.PercentOutput, speed)

    }

    enum class PivotPosition {
        EXTENDED,
        RETRACTED
    }
}
