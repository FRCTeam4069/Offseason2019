package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.elevator.OperatorElevatorCommand
import frc.team4069.robot.control.elevator.ElevatorController
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.hertz
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import frc.team4069.saturn.lib.util.launchFrequency
import kotlinx.coroutines.GlobalScope

object Elevator : SaturnSubsystem() {
    private val masterTalon = SaturnSRX(RobotMap.Elevator.MAIN_SRX, Constants.ELEVATOR_MODEL)
    private val slaveTalon = SaturnSRX(RobotMap.Elevator.SLAVE_SRX, Constants.ELEVATOR_MODEL)


    val MAX_HEIGHT = 31.inch

//    val controller = ElevatorController()

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

        GlobalScope.launchFrequency(50.hertz) {
        }

        slaveTalon.follow(masterTalon)
    }

    override fun lateInit() {
        position = 0.inch
    }

    override fun teleopReset() {
        OperatorElevatorCommand().start()
    }

    fun set(output: Double) {
        masterTalon.setDutyCycle(output)
    }

    fun set(position: Length) {
        masterTalon.setPosition(position)
    }

    var position: Length
        get() = masterTalon.encoder.position
        set(value) {
            masterTalon.encoder.resetPosition(value)
        }

    val velocity: LinearVelocity
        get() = masterTalon.encoder.velocity


    override fun periodic() {
        SmartDashboard.putNumber("Elevator Position", position.inch)
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