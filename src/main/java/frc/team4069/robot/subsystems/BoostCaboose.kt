package frc.team4069.robot.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import frc.team4069.robot.RobotMap
import frc.team4069.robot.commands.climber.DriveClimberCommand
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitSensorModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU
import frc.team4069.saturn.lib.motor.SaturnMAX

object BoostCaboose : SaturnSubsystem() {
    val model = NativeUnitLengthModel(20.49.STU, (2.9 / 2.0).inch)
    val liftMaster = SaturnMAX(RobotMap.Climber.MAIN_MAX, model = model)
    private val liftSlave1 = SaturnMAX(RobotMap.Climber.SLAVE_MAX, model = NativeUnitSensorModel(1.STU))

    private val solenoid = DoubleSolenoid(RobotMap.Climber.SOLENOID_FWD, RobotMap.Climber.SOLENOID_BCK)

    init {
        defaultCommand = DriveClimberCommand()
        liftMaster.apply {
            inverted = true
        }

        liftSlave1.follow(liftMaster)
    }

    fun set(percent: Double) {
        liftMaster.set(percent)
    }

    fun set(dir: DoubleSolenoid.Value) {
        solenoid.set(dir)
    }

    fun toggle() {
        when (solenoid.get()) {
            DoubleSolenoid.Value.kOff, DoubleSolenoid.Value.kReverse -> set(DoubleSolenoid.Value.kForward)
            else -> set(DoubleSolenoid.Value.kReverse)
        }
    }

    override fun teleopReset() {
        DriveClimberCommand().start()
    }
}