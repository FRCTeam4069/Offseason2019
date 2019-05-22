package frc.team4069.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import frc.team4069.robot.Constants
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.drive.OperatorDriveCommand
import frc.team4069.saturn.lib.localization.DeadReckoningLocalization
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STUPer100ms
import frc.team4069.saturn.lib.motor.ctre.SaturnSRX
import frc.team4069.saturn.lib.sensors.SaturnPigeon
import frc.team4069.saturn.lib.subsystem.TankDriveSubsystem
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

object Drivetrain : TankDriveSubsystem(), Loggable {

    private val leftSlave = SaturnSRX(RobotMap.Drivetrain.LEFT_SLAVE_SRX, Constants.DT_MODEL)
    private val rightSlave = SaturnSRX(RobotMap.Drivetrain.RIGHT_SLAVE_SRX, Constants.DT_MODEL)

    override val leftMotor = SaturnSRX(RobotMap.Drivetrain.LEFT_MAIN_SRX, Constants.DT_MODEL)
    override val rightMotor = SaturnSRX(RobotMap.Drivetrain.RIGHT_MAIN_SRX, Constants.DT_MODEL)

    val auxMotor = TalonSRX(RobotMap.Drivetrain.AUXILIARY_MAIN_SRX)
    val auxSlave = TalonSRX(RobotMap.Drivetrain.AUXILIARY_SLAVE_SRX)

    @Log.ToString(name = "Current State", rowIndex = 0, columnIndex = 5)
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    private val periodicIO = PeriodicIO()

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

    override fun tankDrive(left: Double, right: Double) {
        periodicIO.leftDemand = left
        periodicIO.rightDemand = right
        periodicIO.auxDutyCycle = 0.0
        wantedState = State.OpenLoop
    }

    override fun periodic() {
        periodicIO.leftVoltage = leftMotor.voltageOutput
        periodicIO.rightVoltage = rightMotor.voltageOutput
        periodicIO.leftCurrent = leftMotor.talon.outputCurrent
        periodicIO.rightCurrent = rightMotor.talon.outputCurrent
        periodicIO.leftPosition = leftPosition.meter
        periodicIO.rightPosition = rightPosition.meter
        periodicIO.leftVelocity = leftMotor.encoder.velocity.value
        periodicIO.rightVelocity = rightMotor.encoder.velocity.value
        periodicIO.gyroAngle = gyro().degree

        //TODO: Telemetry
        when (wantedState) {
            State.OpenLoop -> {
                leftMotor.setDutyCycle(periodicIO.leftDemand)
                rightMotor.setDutyCycle(periodicIO.rightDemand)

                auxMotor.set(ControlMode.PercentOutput, periodicIO.auxDutyCycle)
            }
            State.PathFollowing -> {
                leftMotor.setVelocity(periodicIO.leftDemand.meter.velocity, periodicIO.leftArbFF)
                rightMotor.setVelocity(periodicIO.rightDemand.meter.velocity, periodicIO.rightArbFF)
            }
            State.MotionMagic -> {
                leftMotor.setPosition(periodicIO.leftDemand.meter)
                rightMotor.setPosition(periodicIO.rightDemand.meter)
            }
            State.Nothing -> {
                leftMotor.setNeutral()
                rightMotor.setNeutral()
                auxMotor.neutralOutput() // One of these things is not like the others
            }
        }
        if (currentState != wantedState) currentState = wantedState
    }

    // Aux dt should only be used with %vbus.
    // Different gearing than normal dt, different motors
    // Not to be used in PID
    fun setAux(vbus: Double, onlyAux: Boolean) {
        periodicIO.auxDutyCycle = vbus
        wantedState = State.OpenLoop
        if (onlyAux) {
            periodicIO.leftDemand = 0.0
            periodicIO.rightDemand = 0.0
        }
    }

    fun set(left: LinearVelocity, right: LinearVelocity, leftAff: Double = 0.0,
            rightAff: Double = 0.0) {
        periodicIO.apply {
            leftDemand = left.value
            rightDemand = right.value
            leftArbFF = leftAff
            rightArbFF = rightAff
        }
        wantedState = State.PathFollowing
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
        periodicIO.apply {
            leftDemand = this@Drivetrain.leftPosition.meter + length.meter
            rightDemand = this@Drivetrain.rightPosition.meter + length.meter
        }
        wantedState = State.MotionMagic
    }

    private class PeriodicIO : Loggable {
        override fun configureLayoutType() = BuiltInLayouts.kGrid
        override fun skipLayout() = true

        @Log.VoltageView(name = "Left Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
        var leftVoltage: Double = 0.0

        @Log.VoltageView(name = "Right Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 2)
        var rightVoltage: Double = 0.0

        @Log(name = "Left Current", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
        var leftCurrent: Double = 0.0

        @Log(name = "Right Current", width = 2, height = 1, rowIndex = 1, columnIndex = 2)
        var rightCurrent: Double = 0.0

        @Log(name = "Left Position (m)", width = 2, height = 1, rowIndex = 2, columnIndex = 0)
        var leftPosition: Double = 0.0

        @Log(name = "Right Position (m)", width = 2, height = 1, rowIndex = 2, columnIndex = 2)
        var rightPosition: Double = 0.0

        @Log(name = "Left Velocity (m/s)", width = 2, height = 1, rowIndex = 3, columnIndex = 0)
        var leftVelocity: Double = 0.0

        @Log(name = "Right Velocity (m/s)", width = 2, height = 1, rowIndex = 3, columnIndex = 2)
        var rightVelocity: Double = 0.0

        @Log(name = "Gyro Angle", rowIndex = 4, columnIndex = 0)
        var gyroAngle: Double = 0.0

        // Outputs
        var leftDemand: Double = 0.0
        var leftArbFF: Double = 0.0
        var rightDemand: Double = 0.0
        var rightArbFF: Double = 0.0

        var auxDutyCycle: Double = 0.0
    }

    private enum class State {
        OpenLoop,
        PathFollowing,
        MotionMagic,
        Nothing
    }
}
