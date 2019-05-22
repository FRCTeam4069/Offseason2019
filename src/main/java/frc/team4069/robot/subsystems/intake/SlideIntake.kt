package frc.team4069.robot.subsystems.intake

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.intake.AlignHatchIntakeCommand
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.inchesPerSecond
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

object SlideIntake : SaturnSubsystem(), Loggable {
    private val slide = SaturnSRX(RobotMap.Intake.SLIDE_SRX, Constants.INTAKE_SLIDE_MODEL)

    private val telemetry = SubsystemTelemetry()

    @Log(name = "Current State", rowIndex = 0, columnIndex = 3)
    private var currentState: State = State.Nothing
    private var wantedState: State = State.Nothing

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
    }

    override fun lateInit() {
        slide.encoder.resetPosition(0.inch)
    }

    override fun teleopReset() {
        AlignHatchIntakeCommand().start()
    }

    override fun periodic() {
        telemetry.voltage = slide.voltageOutput
        telemetry.current = slide.talon.outputCurrent
        telemetry.position = slide.encoder.position.inch
        telemetry.velocity = slide.encoder.velocity.inchesPerSecond

        when(val state = wantedState) {
            is State.OpenLoop -> {
                slide.setDutyCycle(state.dutyCycle)
            }
            is State.MotionMagic -> {
                slide.setPosition(state.setpoint)
            }
        }
        if(currentState != wantedState) currentState = wantedState
    }

    fun setDutyCycle(dutyCycle: Double) {
        wantedState = State.OpenLoop(dutyCycle)
    }

    fun setPosition(position: Length) {
        wantedState = State.MotionMagic(position)
    }

    private class SubsystemTelemetry : Loggable {
        override fun configureLayoutType() = BuiltInLayouts.kGrid
        override fun skipLayout() = true

        @Log.VoltageView(name = "Voltage", rowIndex = 0, columnIndex = 0)
        var voltage = 0.0

        @Log(name = "Current", rowIndex = 1, columnIndex = 0)
        var current = 0.0

        @Log(name = "Position (in)", rowIndex = 0, columnIndex = 1)
        var position = 0.0

        @Log(name = "Velocity (in/s)", rowIndex = 1, columnIndex = 1)
        var velocity = 0.0
    }

    private sealed class State {
        data class OpenLoop(
                @Log(name = "Duty cycle", rowIndex = 0, columnIndex = 0)
                val dutyCycle: Double
        ) : State(), Loggable {
            override fun configureLayoutType() = BuiltInLayouts.kGrid
            override fun skipLayout() = true
        }

        data class MotionMagic(val setpoint: Length) : State(), Loggable {
            override fun configureLayoutType() = BuiltInLayouts.kGrid
            override fun skipLayout() = true

            @Log(name = "Setpoint (in)", rowIndex = 0, columnIndex = 0)
            private val _setpoint = setpoint.inch
        }

        object Nothing : State(), Loggable {
            override fun toString() = "Nothing"
        }
    }
}