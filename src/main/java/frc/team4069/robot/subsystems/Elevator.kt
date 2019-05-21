package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.elevator.OperatorElevatorCommand
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.amp
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.meter
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.mathematics.units.nativeunits.fromModel
import frc.team4069.saturn.lib.motor.NativeSaturnSRX
import frc.team4069.saturn.lib.motor.SaturnSRX

object Elevator : SaturnSubsystem() {
    private val masterTalon = SaturnSRX(RobotMap.Elevator.MAIN_SRX, Constants.ELEVATOR_MODEL)
    private val slaveTalon = NativeSaturnSRX(RobotMap.Elevator.SLAVE_SRX)

    val MAX_HEIGHT = 31.inch

    init {
        defaultCommand = OperatorElevatorCommand()
        val maxVel = 1.133.meter.velocity.fromModel(Constants.ELEVATOR_MODEL)
        masterTalon.apply {
            continuousCurrentLimit = 35.amp
            currentLimitingEnabled = true
            inverted = true
            kP = 0.5
            kF = 1023.0 / maxVel.STUPer100ms
            kD = 0.05
            motionCruiseVelocity = 30.inch.velocity
            motionAcceleration = 45.inch.acceleration
//            allowedClosedLoopError = 0.8.inch
            softLimitForward = MAX_HEIGHT
            softLimitReverse = 0.inch
            softLimitForwardEnabled = true
            //softLimitReverseEnabled = true
            setSensorPhase(true)
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
        masterTalon.set(ControlMode.PercentOutput, output)
    }

    fun set(position: Length) {
        masterTalon.set(ControlMode.MotionMagic, position)
    }

    var position: Length
        get() = masterTalon.sensorPosition
        set(value) {
            masterTalon.sensorPosition = value
        }

    val velocity: LinearVelocity
        get() = masterTalon.sensorVelocity


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