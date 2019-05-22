package frc.team4069.robot

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team4069.robot.commands.drive.auto.RocketShipLeftAuto
import frc.team4069.robot.commands.drive.auto.RocketShipRightAuto
import frc.team4069.robot.commands.drive.auto.RocketShipRightBackwardsAuto
import frc.team4069.robot.commands.drive.auto.TeleoperatedSandstorm
import frc.team4069.robot.subsystems.BoostCaboose
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.robot.vision.VisionSystem
import frc.team4069.saturn.lib.SaturnRobot
import frc.team4069.saturn.lib.commands.SaturnCommandGroup
import frc.team4069.saturn.lib.commands.SaturnSubsystem
import frc.team4069.saturn.lib.shuffleboard.chooser
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.Logger
import kotlinx.coroutines.ObsoleteCoroutinesApi

object Robot : SaturnRobot() {
    private var brownedOut = false

    private val compressor = Compressor()

    private lateinit var autoChooser: SendableChooser<SaturnCommandGroup>

    val loggableSystems = arrayListOf<Loggable>()

    @ObsoleteCoroutinesApi
    override fun initialize() {
        // Subsystem initialization
        +Drivetrain
        +BoostCaboose
        +Intake
        +Elevator

        // Controls registering
        +OI.driveController
        +OI.operatorController

        // Helper initialization
        Trajectories
        VisionSystem

        SmartDashboard.putBoolean("Voltage Nominal", true)

        autoChooser = chooser("Autonomous Selection") {
            "Rocket Right" += RocketShipRightAuto()
            "Rocket Left" += RocketShipLeftAuto()
            "Rocket Right (Backwards)" += RocketShipRightBackwardsAuto()
            "Teleoperated Sandstorm" += TeleoperatedSandstorm()
        }

        Logger.configureLoggingAndConfig(this, false)
    }

    /**
     * Function called when brownout watchdog has triggered. Robot is running at dangerously low voltage,
     * and compensation is required to get through the match. Stop non-essential systems, reduce current limits, and notify drivers
     */
    override fun notifyBrownout() {
        SmartDashboard.putBoolean("Voltage Nominal", false)
        compressor.stop()
        brownedOut = true
    }

    override fun autonomousInit() {
        autoChooser.selected.start()
    }

    operator fun SaturnSubsystem.unaryPlus() {
        addToSubsystemHandler(this)

        if(this is Loggable) {
            loggableSystems.add(this)
        }
    }
}

/**
 * Entry point for robot code. Don't touch
 * Initialization should go in Robot.initialize()
 */
fun main() {
    RobotBase.startRobot { Robot }
}
