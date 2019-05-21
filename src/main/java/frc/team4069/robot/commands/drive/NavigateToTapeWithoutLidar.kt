package frc.team4069.robot.commands.drive

import frc.team4069.robot.Constants
import frc.team4069.robot.Trajectories
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.robot.util.PathKinematicsHelper
import frc.team4069.robot.vision.VisionSystem
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.debug.LiveDashboard
import frc.team4069.saturn.lib.mathematics.twodim.control.RamseteController
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import frc.team4069.saturn.lib.mathematics.units.Rotation2d
import frc.team4069.saturn.lib.mathematics.units.degree
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.feet
import frc.team4069.saturn.lib.mathematics.units.inch
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.ObsoleteCoroutinesApi
import kotlinx.coroutines.newSingleThreadContext
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

/**
 * Class encapsulating functionality to navigate to tape target without using the LiDAR
 * In this case, distance to the target will be calculated purely with the camera, using solvePnP
 */
@ObsoleteCoroutinesApi
class NavigateToTapeWithoutLidar(val endRot: Rotation2d = 0.degree) : SaturnCommand(Drivetrain) {
    private lateinit var follower: RamseteController

    private val kinematicsHelper = PathKinematicsHelper()

    init {
        finishCondition += { follower.isFinished }
        // Run in its own coroutine for better timing
//        executeFrequency = 50
    }

    override suspend fun initialize() {
        println("Navigating to tape")
        LiveDashboard.isFollowingPath = true

        while (true) {
            // Z is image depth, X is lateral. Correspond to field-relative x and y respectively
            val targetX = VisionSystem.targetZ ?: continue
            val targetY = -(VisionSystem.targetX ?: continue) // lul
            val endTargetPose = Pose2d(targetX, targetY * (endRot - Drivetrain.robotPosition.rotation).sin, endRot) + Trajectories.cameraToIntake

            val path = Trajectories.navigateToRelativePosition(relativeEndingPose = endTargetPose,
                    constraints = listOf(
                            CentripetalAccelerationConstraint(0.5.feet.acceleration)
                    ),
                    maxVelocity = 0.75.feet.velocity,
                    reversed = false)

            follower = RamseteController(path, Constants.kB, Constants.kZeta)

            break
        }
    }

    override suspend fun execute() {
        val currentPose = Drivetrain.robotPosition
        val state = follower.update(currentPose)

        val (left, right, laff, raff) = kinematicsHelper.update(state)

//        Drivetrain.set(left,
//                right,
//                laff,
//                raff)
        updateDashboard()
    }

    override suspend fun dispose() {
        Drivetrain.stop()
        LiveDashboard.isFollowingPath = false
        val pose = Drivetrain.robotPosition
        println("Final pose: ${pose.translation.x.inch}, ${pose.translation.y.inch}, ${pose.rotation.degree}")
    }

    private fun updateDashboard() {
        val seg = follower.referencePose
        LiveDashboard.pathX = seg.translation.x.feet
        LiveDashboard.pathY = seg.translation.y.feet
        LiveDashboard.pathHeading = seg.rotation.radian
    }


}