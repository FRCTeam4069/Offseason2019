package frc.team4069.robot.subsystems

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.climber.DriveClimberCommand
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitSensorModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.motor.rev.SaturnMAX
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log

object BoostCaboose : SaturnSubsystem(), Loggable {
    val model = NativeUnitLengthModel(20.49.STU, (2.9 / 2.0).inch)
    val liftMaster = SaturnMAX(RobotMap.Climber.MAIN_MAX, CANSparkMaxLowLevel.MotorType.kBrushless, model = model)
    private val liftSlave1 = SaturnMAX(RobotMap.Climber.SLAVE_MAX, CANSparkMaxLowLevel.MotorType.kBrushless, model = NativeUnitSensorModel(1.STU))

    private val solenoid = DoubleSolenoid(RobotMap.Climber.SOLENOID_FWD, RobotMap.Climber.SOLENOID_BCK)

    private val periodicIO = PeriodicIO()

    init {
        defaultCommand = DriveClimberCommand()
        liftMaster.apply {
            outputInverted = true
        }

        liftSlave1.follow(liftMaster)
    }

    fun set(percent: Double) {
        periodicIO.demand = percent
    }

    fun set(dir: DoubleSolenoid.Value) {
        periodicIO.pistonState = dir == DoubleSolenoid.Value.kForward

        solenoid.set(dir)
    }

    override fun periodic() {
        periodicIO.voltage = liftMaster.voltageOutput
        periodicIO.current = liftMaster.canSparkMax.outputCurrent

        liftMaster.setDutyCycle(periodicIO.demand)
    }

    fun toggle() {
        when (solenoid.get()) {
            DoubleSolenoid.Value.kOff, DoubleSolenoid.Value.kReverse -> set(DoubleSolenoid.Value.kForward)
            else -> set(DoubleSolenoid.Value.kReverse)
        }
        periodicIO.pistonState = !periodicIO.pistonState
    }

    private class PeriodicIO : Loggable {
        override fun configureLayoutType() = BuiltInLayouts.kGrid
        override fun skipLayout() = true

        @Log.VoltageView(name = "Voltage", rowIndex = 0, columnIndex = 0)
        var voltage: Double = 0.0

        @Log(name = "Current", rowIndex = 1, columnIndex = 0)
        var current: Double = 0.0

        // Outputs
        var demand: Double = 0.0
        var pistonState: Boolean = false
    }

    override fun teleopReset() {
        DriveClimberCommand().schedule()
    }
}