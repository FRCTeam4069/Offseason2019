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
import frc.team4069.saturn.lib.mathematics.units.meter
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

object SlideIntake : SaturnSubsystem(), Loggable {
    private val slide = SaturnSRX(RobotMap.Intake.SLIDE_SRX, Constants.INTAKE_SLIDE_MODEL)

    private val periodicIO = PeriodicIO()

    @Log.ToString(name = "Current State", rowIndex = 0, columnIndex = 3)
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

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
        periodicIO.voltage = slide.voltageOutput
        periodicIO.current = slide.talon.outputCurrent
        periodicIO.position = slide.encoder.position.inch
        periodicIO.velocity = slide.encoder.velocity.inchesPerSecond

        when(wantedState) {
            State.OpenLoop -> {
                slide.setDutyCycle(periodicIO.demand)
            }
            State.MotionMagic -> {
                slide.setPosition(periodicIO.demand.meter)
            }
        }
        if(currentState != wantedState) currentState = wantedState
    }

    fun setDutyCycle(dutyCycle: Double) {
        wantedState = State.OpenLoop
        periodicIO.demand = dutyCycle
    }

    fun setPosition(position: Length) {
        wantedState = State.MotionMagic
        periodicIO.demand = position.meter
    }

    private class PeriodicIO : Loggable {
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

        // Outputs
        var demand: Double = 0.0
    }

    private enum class State {
        OpenLoop,
        MotionMagic,
        Nothing
    }
}