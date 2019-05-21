package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
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
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import frc.team4069.saturn.lib.sensors.SaturnPigeon
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem

object Drivetrain : TankDriveSubsystem() {

    private val leftSlave = SaturnSRX(RobotMap.Drivetrain.LEFT_SLAVE_SRX, Constants.DT_MODEL)
    private val rightSlave = SaturnSRX(RobotMap.Drivetrain.RIGHT_SLAVE_SRX, Constants.DT_MODEL)

    override val leftMotor = SaturnSRX(RobotMap.Drivetrain.LEFT_MAIN_SRX, Constants.DT_MODEL)
    override val rightMotor = SaturnSRX(RobotMap.Drivetrain.RIGHT_MAIN_SRX, Constants.DT_MODEL)

    val auxMotor = TalonSRX(RobotMap.Drivetrain.AUXILIARY_MAIN_SRX)
    val auxSlave = TalonSRX(RobotMap.Drivetrain.AUXILIARY_SLAVE_SRX)

    override val gyro = SaturnPigeon(leftSlave.talon)

    override val localization = DeadReckoningLocalization(gyro, { leftPosition }, { rightPosition })

    val leftPosition: Length
        get() = leftMotor.encoder.position

    val rightPosition: Length
        get() = rightMotor.encoder.position

    init {
        // kV is 1/m/s. Invert to get m/s. Convert to STU
        val maxVelL = Constants.DT_MODEL.toNativeUnitVelocity((1 / Constants.DT_LEFT_KV).meter.velocity)
        val maxVelR = Constants.DT_MODEL.toNativeUnitVelocity((1 / Constants.DT_RIGHT_KV).meter.velocity)

        leftMotor.apply {
            configCurrentLimit(true, SaturnSRX.CurrentLimitConfig(
                    continuousCurrentLimit = 50.amp,
                    peakCurrentLimit = 70.amp,
                    peakCurrentLimitDuration = 1.75.second
            ))

            encoder.encoderPhase = true

            motionProfileAcceleration = 3.5.feet.acceleration
            motionProfileCruiseVelocity = 3.feet.velocity
            useMotionProfileForPosition = true
            talon.setNeutralMode(NeutralMode.Brake)

            talon.config_kF(0, 1023.0 / maxVelL.STUPer100ms)
            talon.config_kP(0, 0.7)
            talon.config_kD(0, 0.05)
        }

        rightMotor.apply {
            configCurrentLimit(true, SaturnSRX.CurrentLimitConfig(
                    continuousCurrentLimit = 50.amp,
                    peakCurrentLimit = 70.amp,
                    peakCurrentLimitDuration = 1.75.second
            ))

            motionProfileAcceleration = 3.5.feet.acceleration
            motionProfileCruiseVelocity = 3.feet.velocity
            useMotionProfileForPosition = true

            outputInverted = true
            talon.setNeutralMode(NeutralMode.Brake)
            encoder.encoderPhase = true

            talon.config_kF(0, 1023.0 / maxVelR.STUPer100ms)
            talon.config_kP(0, 1.0)
            talon.config_kD(0, 0.01)
        }

        leftSlave.follow(leftMotor)
        leftSlave.talon.setNeutralMode(NeutralMode.Coast)
        rightSlave.outputInverted = true
        rightSlave.follow(rightMotor)
        rightSlave.talon.setNeutralMode(NeutralMode.Coast)

        auxSlave.inverted = true
        auxSlave.follow(auxMotor)

    }

    fun stop() {
        leftMotor.setNeutral()
        rightMotor.setNeutral()
    }

    fun reset() {
        stop()
        leftMotor.encoder.resetPosition(0.meter)
        rightMotor.encoder.resetPosition(0.meter)
        gyro.setFusedHeading(0.0)
    }

    override fun lateInit() {
        super.lateInit()
        reset()
    }

    override fun teleopReset() {
        OperatorDriveCommand().start()
    }

    // Aux dt should only be used with %vbus.
    // Different gearing than normal dt, different motors
    // Not to be used in PID
    fun setAux(vbus: Double) {
        auxMotor.set(ControlMode.PercentOutput, vbus)
    }

    fun set(left: LinearVelocity, right: LinearVelocity, leftAff: Double = 0.0,
            rightAff: Double = 0.0) {

        leftMotor.setVelocity(left, leftAff)
        rightMotor.setVelocity(right, rightAff)
    }

    fun reduceLimits() {
        leftMotor.configCurrentLimit(true, SaturnSRX.CurrentLimitConfig(
                peakCurrentLimitDuration = 1.75.second,
                peakCurrentLimit = 70.amp,
                continuousCurrentLimit = 28.amp
        ))

        rightMotor.configCurrentLimit(true, SaturnSRX.CurrentLimitConfig(
                peakCurrentLimitDuration = 1.75.second,
                peakCurrentLimit = 70.amp,
                continuousCurrentLimit = 28.amp
        ))
    }

    fun motionMagicRelative(length: Length) {
        leftMotor.setPosition(leftPosition + length)
        rightMotor.setPosition(rightPosition + length)
    }
}
