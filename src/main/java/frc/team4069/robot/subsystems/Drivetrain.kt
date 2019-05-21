package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.NeutralMode
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.drive.OperatorDriveCommand
import frc.team4069.saturn.lib.localization.DeadReckoningLocalization
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.mathematics.units.nativeunits.fromModel
import frc.team4069.saturn.lib.motor.NativeSaturnSRX
import frc.team4069.saturn.lib.motor.SaturnSRX
import frc.team4069.saturn.lib.sensors.SaturnPigeon
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem

object Drivetrain : TankDriveSubsystem() {

    private val leftSlave = SaturnSRX(RobotMap.Drivetrain.LEFT_SLAVE_SRX, Constants.DT_MODEL)
    private val rightSlave = SaturnSRX(RobotMap.Drivetrain.RIGHT_SLAVE_SRX, Constants.DT_MODEL)

    override val leftMotor = SaturnSRX(RobotMap.Drivetrain.LEFT_MAIN_SRX, Constants.DT_MODEL)
    override val rightMotor = SaturnSRX(RobotMap.Drivetrain.RIGHT_MAIN_SRX, Constants.DT_MODEL)

    val auxMotor = NativeSaturnSRX(RobotMap.Drivetrain.AUXILIARY_MAIN_SRX, 4096.STU) // Unsensored
    val auxSlave = NativeSaturnSRX(RobotMap.Drivetrain.AUXILIARY_SLAVE_SRX, 4096.STU)

    override val gyro = SaturnPigeon(leftSlave)

    override val localization = DeadReckoningLocalization(gyro, { leftPosition }, { rightPosition })

    val leftPosition: Length
        get() = leftMotor.sensorPosition

    val rightPosition: Length
        get() = rightMotor.sensorPosition

    init {
        // kV is 1/m/s. Invert to get m/s. Convert to STU
        val maxVelL = (1 / Constants.DT_LEFT_KV).meter.velocity.fromModel(Constants.DT_MODEL)
        val maxVelR = (1 / Constants.DT_RIGHT_KV).meter.velocity.fromModel(Constants.DT_MODEL)

        leftMotor.apply {
            continuousCurrentLimit = 50.amp
            peakCurrentLimit = 70.amp
            peakCurrentLimitDuration = 1.75.second
            currentLimitingEnabled = true
            motionAcceleration = 3.5.feet.acceleration
            motionCruiseVelocity = 3.feet.velocity
            setNeutralMode(NeutralMode.Brake)
            setSensorPhase(true)

            kF = 1023.0 / maxVelL.STUPer100ms
            kP = 0.7
            kD = 0.05
        }

        rightMotor.apply {
            continuousCurrentLimit = 50.amp
            peakCurrentLimit = 70.amp
            peakCurrentLimitDuration = 1.75.second
            currentLimitingEnabled = true
            motionAcceleration = 3.5.feet.acceleration
            motionCruiseVelocity = 3.feet.velocity
            inverted = true
            setNeutralMode(NeutralMode.Brake)
            setSensorPhase(true)
//            kF = 1023.0 * Constants.DT_RIGHT_KV.fromModel(Constants.DT_MODEL).STUPer100ms
            kF = 1023.0 / maxVelR.STUPer100ms
            kP = 1.0
            kD = 0.01
        }

        leftSlave.follow(leftMotor)
        leftSlave.setNeutralMode(NeutralMode.Coast)
        rightSlave.inverted = true
        rightSlave.follow(rightMotor)
        rightSlave.setNeutralMode(NeutralMode.Coast)

        auxSlave.inverted = true
        auxSlave.follow(auxMotor)

    }

    fun stop() {
        leftMotor.neutralOutput()
        rightMotor.neutralOutput()
    }

    fun reset() {
        stop()
        leftMotor.sensorPosition = 0.meter
        rightMotor.sensorPosition = 0.meter
        gyro.setFusedHeading(0.0)
    }

    override fun lateInit() {
        super.lateInit()
        reset()
    }

    override fun teleopReset() {
        OperatorDriveCommand().start()
    }

    fun set(mode: ControlMode, left: Double, right: Double) {
        leftMotor.set(mode, left)
        rightMotor.set(mode, right)
    }

    // Aux dt should only be used with %vbus.
    // Different gearing than normal dt, different motors
    // Not to be used in PID
    fun setAux(vbus: Double) {
        auxMotor.set(ControlMode.PercentOutput, vbus)
    }

    fun set(left: LinearVelocity, right: LinearVelocity, leftAff: Double = 0.0,
            rightAff: Double = 0.0) {

        leftMotor.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, leftAff)
        rightMotor.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, rightAff)
    }

    fun reduceLimits() {
        leftMotor.apply {
            currentLimitingEnabled = false
            continuousCurrentLimit = 28.amp
            currentLimitingEnabled = true
        }

        rightMotor.apply {
            currentLimitingEnabled = false
            continuousCurrentLimit = 28.amp
            currentLimitingEnabled = true
        }
    }

    fun motionMagicRelative(length: Length) {
        leftMotor.set(ControlMode.MotionMagic, leftPosition + length)
        rightMotor.set(ControlMode.MotionMagic, rightPosition + length)
    }
}
