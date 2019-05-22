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

    private var currentState: State = State.Nothing
    private var wantedState: State = State.Nothing

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
        wantedState = State.OpenLoop(left, right)
    }

    override fun periodic() {
        //TODO: Telemetry
        when(val state = wantedState) {
            is State.OpenLoop -> {
                leftMotor.setDutyCycle(state.left)
                rightMotor.setDutyCycle(state.right)

                if(state.aux != null) {
                    auxMotor.set(ControlMode.PercentOutput, state.aux!!) // state is an immutable copy, compiler not smart enough that I can remove !!
                }
            }
            is State.AuxOnly -> {
                auxMotor.set(ControlMode.PercentOutput, state.demand)
            }
            is State.PathFollowing -> {
                leftMotor.setVelocity(state.leftVelocity, state.leftArbFF)
                rightMotor.setVelocity(state.rightVelocity, state.rightArbFF)
            }
            is State.MotionMagic -> {
                leftMotor.setPosition(state.leftSetpoint)
                rightMotor.setPosition(state.rightSetpoint)
            }
            is State.Nothing -> {
                leftMotor.setNeutral()
                rightMotor.setNeutral()
                auxMotor.neutralOutput() // One of these things is not like the others
            }
        }
        if(currentState != wantedState) currentState = wantedState
    }

    // Aux dt should only be used with %vbus.
    // Different gearing than normal dt, different motors
    // Not to be used in PID
    fun setAux(vbus: Double) {
        if(wantedState is State.OpenLoop) {
            (wantedState as State.OpenLoop).aux = vbus // Command locking the subsystem means this shouldn't fail
        }else {
            wantedState = State.AuxOnly(vbus)
        }
    }

    fun set(left: LinearVelocity, right: LinearVelocity, leftAff: Double = 0.0,
            rightAff: Double = 0.0) {

        wantedState = State.PathFollowing(left, right, leftAff, rightAff)
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
        wantedState = State.MotionMagic(leftPosition + length, rightPosition + length)
    }

    private sealed class State : Loggable {
        override fun configureLayoutType() = BuiltInLayouts.kGrid
        override fun skipLayout() = true

        data class OpenLoop(
                @Log(name = "Left Output", rowIndex = 0, columnIndex = 0)
                val left: Double,
                @Log(name = "Right Output", rowIndex = 1, columnIndex = 0)
                val right: Double,
                @Log(name = "Back Output", rowIndex = 2, columnIndex = 0)
                var aux: Double? = null) : State()

        data class AuxOnly(
                @Log(name = "Back Output", rowIndex = 0, columnIndex = 0)
                val demand: Double) : State()

        data class MotionMagic(val leftSetpoint: Length, val rightSetpoint: Length) : State() {
            @Log(name = "Left Setpoint (m)", rowIndex = 0, columnIndex = 0)
            private val _leftSetpoint = leftSetpoint.meter

            @Log(name = "Right Setpoint (m)", rowIndex = 1, columnIndex = 0)
            private val _rightSetpoint = rightSetpoint.meter
        }

        data class PathFollowing(
                val leftVelocity: LinearVelocity,
                val rightVelocity: LinearVelocity,
                @Log(name = "Left Arbitrary Feedforward", rowIndex = 0, columnIndex = 1)
                val leftArbFF: Double,
                @Log(name = "Right Arbitrary Feedforward", rowIndex = 1, columnIndex = 1)
                val rightArbFF: Double
        ) : State() {
            @Log(name = "Left Velocity (m/s)", rowIndex = 0, columnIndex = 0)
            private val _leftVelocity = leftVelocity.value
            @Log(name = "Right Velocity (m/s)", rowIndex = 1, columnIndex = 0)
            private val _rightVelocity = rightVelocity.value
        }

        object Nothing : State() {
            override fun toString() = "Nothing"
        }
    }
}
