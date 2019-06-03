package frc.team4069.robot.subsystems

import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.elevator.OperatorElevatorCommand
import frc.team4069.robot.control.elevator.ElevatorController
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.inchesPerSecond
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import frc.team4069.saturn.lib.mathematics.onedim.control.TrapezoidalProfile
import kotlin.math.abs

object Elevator : SaturnSubsystem(), Loggable {
    private val masterTalon = SaturnSRX(RobotMap.Elevator.MAIN_SRX, Constants.ELEVATOR_MODEL)
    private val slaveTalon = SaturnSRX(RobotMap.Elevator.SLAVE_SRX, Constants.ELEVATOR_MODEL)

    private val periodicIO = PeriodicIO()

    @Log.ToString(name = "Current state", rowIndex = 0, columnIndex = 3)
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    val MAX_HEIGHT = 31.inch

    val controller = ElevatorController()

    init {
        defaultCommand = OperatorElevatorCommand()
        val maxVel = Constants.ELEVATOR_MODEL.toNativeUnitVelocity(1.133.meter.velocity)
                .STUPer100ms
        masterTalon.apply {
            configCurrentLimit(true, SaturnSRX.CurrentLimitConfig(
                    peakCurrentLimit = 0.amp,
                    peakCurrentLimitDuration = 0.second,
                    continuousCurrentLimit = 35.amp
            ))
            outputInverted = true
            talon.config_kP(0, 0.5)
            talon.config_kF(0, 1023.0 / maxVel)
            talon.config_kD(0, 0.05)

            motionProfileCruiseVelocity = 30.inch.velocity
            motionProfileAcceleration = 45.inch.acceleration
            useMotionProfileForPosition = true
//            allowedClosedLoopError = 0.8.inch
            talon.configForwardSoftLimitThreshold(Constants.ELEVATOR_MODEL.toNativeUnitPosition(MAX_HEIGHT).value.toInt())
            talon.configReverseSoftLimitThreshold(0)
            talon.configForwardSoftLimitEnable(true)
            encoder.encoderPhase = true
        }

        slaveTalon.follow(masterTalon)

        Notifier(this::update).startPeriodic(1.0/100.0)
    }

    override fun lateInit() {
        masterTalon.encoder.resetPosition(0.inch)
    }

    override fun teleopReset() {
        OperatorElevatorCommand().start()
    }

    fun setDutyCycle(output: Double) {
        periodicIO.demand = output
        wantedState = State.OpenLoop
    }

    fun setPosition(targetPosition: Length) {
        periodicIO.demand = targetPosition.meter
        controller.motionProfile = if(targetPosition < position && abs(targetPosition.inch - position.inch) > 5.0) {
            TrapezoidalProfile(targetPosition, 15.inch.velocity, 30.inch.acceleration, initialX = position) // Dont go as fast going down cause gravity
        }else {
            TrapezoidalProfile(targetPosition, 30.inch.velocity, 45.inch.acceleration,
                initialX = position)
        }

        wantedState = State.ClosedLoop
    }

    val position: Length
        get() = masterTalon.encoder.position

    val velocity: LinearVelocity
        get() = masterTalon.encoder.velocity


    private fun update() {
        periodicIO.voltage = masterTalon.voltageOutput
        periodicIO.current = masterTalon.talon.outputCurrent
        periodicIO.position = position.inch
        periodicIO.velocity = velocity.inchesPerSecond
        periodicIO.kfPosition = controller.position.inch
        periodicIO.kfVelocity = controller.velocity.inchesPerSecond

        controller.measuredPosition = position
        controller.measuredVelocity = velocity
        controller.update()

        when (wantedState) {
            State.OpenLoop -> {
                controller.motionProfile = null
                masterTalon.setDutyCycle(periodicIO.demand)
            }
            State.MotionMagic -> {
                controller.motionProfile = null
                masterTalon.setPosition(periodicIO.demand.meter)
            }
            State.Nothing -> {
                controller.motionProfile = null
                masterTalon.setNeutral()
            }
            State.ClosedLoop -> {
                masterTalon.setVoltage(controller.voltage)
//                controller.reference = vec(`2`).fill(periodicIO.demand, 0.0)
            }

        }

        if (currentState != wantedState) currentState = wantedState
    }

    private class PeriodicIO : Loggable {
        override fun configureLayoutType() = BuiltInLayouts.kGrid
        override fun skipLayout() = true

        @Log.VoltageView(name = "Voltage", rowIndex = 0, columnIndex = 0)
        var voltage: Double = 0.0

        @Log(name = "Current", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
        var current: Double = 0.0

        @Log(name = "Position (in)", width = 2, height = 1, rowIndex = 2, columnIndex = 0)
        var position: Double = 0.0

        @Log(name = "KF Estimated Position (in)", width = 2, height = 1, rowIndex = 2, columnIndex = 2)
        var kfPosition: Double = 0.0

        @Log(name = "Velocity (in s^-1)", width = 2, height = 1, rowIndex = 3, columnIndex = 0)
        var velocity: Double = 0.0

        @Log(name = "KF Estimated Velocity (in s^-1)", width = 2, height = 1, rowIndex = 3, columnIndex = 2)
        var kfVelocity: Double = 0.0

        // Outputs
        var demand: Double = 0.0
    }

    private enum class State {
        OpenLoop,
        MotionMagic,
        ClosedLoop,
        Nothing
    }

    /**
     * Preset positions of the elevator
     *
     * The provided length is the distance of the powered (and sensored) first stage of the elevator.
     * The carriage of the elevator goes a lot higher
     */
    enum class Position(val length: Length) {
        LOW_ROCKET_CARGO_HATCH(3.73.inch),
        MID_ROCKET_CARGO_HATCH(19.1.inch),
        HIGH_ROCKET_CARGO_HATCH(30.4.inch),
        CARGO_SHIP_HATCH(1.5.inch), //TODO: Fill me in
        PORTAL_INTAKE(2.4.inch),
        MINIMUM(1.1.inch)
    }
}