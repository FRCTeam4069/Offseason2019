package frc.team4069.robot.commands.drive

import frc.team4069.robot.Constants
import frc.team4069.robot.Trajectories
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.util.PathKinematicsHelper
import frc.team4069.robot.vision.VisionSystem
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.debug.LiveDashboard
import frc.team4069.saturn.lib.mathematics.twodim.control.RamseteController
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.Rotation2d
import frc.team4069.saturn.lib.mathematics.units.derivedunits.feetPerSecond
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.feet
import kotlin.math.absoluteValue

class DriveStraightToTape(val targetEndRotation: Rotation2d? = null) : SaturnCommand(Drivetrain) {

    private lateinit var follower: RamseteController
    private val kinematicsHelper = PathKinematicsHelper()

    init {
        finishCondition += { follower.isFinished }
    }


    override suspend fun initialize() {
        while (true) {
            val dist = (VisionSystem.targetZ ?: continue) + Trajectories.cameraToIntake.translation.x
            follower = RamseteController(Trajectories.trapezoidToDist(dist = dist, maxVelocity = 1.feet.velocity), b = Constants.kB, zeta = Constants.kZeta)
            break
        }
    }

    override suspend fun execute() {
        println("Trying to move")
        val currentPose = Drivetrain.robotPosition
        val state = follower.update(currentPose)
        val (left, right, laff, raff) = kinematicsHelper.update(state)

        Drivetrain.set(left, right, laff, raff)
    }

    override suspend fun dispose() {
        Drivetrain.stop()
    }
}