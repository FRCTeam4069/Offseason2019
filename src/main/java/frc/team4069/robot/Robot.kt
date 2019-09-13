package frc.team4069.robot

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team4069.robot.subsystems.BoostCaboose
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.robot.subsystems.intake.SlideIntake
import frc.team4069.robot.util.PressureSensor
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.hid.SaturnHID
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.Logger
import kotlinx.coroutines.ObsoleteCoroutinesApi

object Robot : SaturnRobot() {
    private var brownedOut = false

    private val compressor = Compressor()
    val pressureSensor = PressureSensor(0)

    val loggableSystems = arrayListOf<Loggable>()
    val controllers = arrayListOf<SaturnHID<*>>()

    override fun robotInit() {
        // Subsystem initialization
        +Drivetrain
        +BoostCaboose
        +Intake
        +SlideIntake
        +Elevator

        // Controls registering
        +OI.driveController
        +OI.operatorController

        // Helper initialization
//        Trajectories
//        VisionSystem
//
//        SmartDashboard.putBoolean("Voltage Nominal", true)

        Logger.setCycleWarningsEnabled(false) // Kotlin `object`s have cyclic references in bytecode that make oblog unhappy
        Logger.configureLoggingAndConfig(this, false)
    }

    override fun robotPeriodic() {
        Logger.updateEntries()

        if (pressureSensor.pressure < 70.0) {
            compressor.start()
        } else if (pressureSensor.pressure >= 90.0) {
            compressor.stop()
        }

        controllers.forEach(SaturnHID<*>::update)
    }

    /**
     * Function called when brownout watchdog has triggered. Robot is running at dangerously low voltage,
     * and compensation is required to get through the match. Stop non-essential systems, reduce current limits, and notify drivers
     */
//    override fun notifyBrownout() {
//        SmartDashboard.putBoolean("Voltage Nominal", false)
//        compressor.stop()
//        brownedOut = true
//    }

    override fun autonomousInit() {
//        autoChooser.selected.srt()
    }

    private operator fun SaturnHID<*>.unaryPlus() {
        controllers.add(this)
    }

    override operator fun SaturnSubsystem.unaryPlus() {
        addToSubsystemHandler(this)

        if (this is Loggable) {
            loggableSystems.add(this)
        }
    }
}

/**
 * Entry point for robot code. Don't touch
 * Initialization should go in Robot.initialize()
 */
fun main() {
    Robot.start()
}
