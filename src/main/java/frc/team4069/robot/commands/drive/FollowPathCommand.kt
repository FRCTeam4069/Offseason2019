package frc.team4069.robot.commands.drive

import frc.team4069.robot.Constants
import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.util.PathKinematicsHelper
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.debug.LiveDashboard
import frc.team4069.saturn.lib.mathematics.twodim.control.RamseteController
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Rectangle2d
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.types.TimedTrajectory

/**
 * Tracks the given path using the Ramsete path following algorithm
 */
class FollowPathCommand(
        val path: TimedTrajectory<Pose2dWithCurvature>,
        val zeroPose: Boolean = false
) : SaturnCommand(Drivetrain) {
    val follower = RamseteController(path, Constants.kB, Constants.kZeta)
    private val kinematicsHelper = PathKinematicsHelper()

    init {
        println("Ramsete starting")
        finishCondition += { follower.isFinished }
    }

    fun markers(block: MarkerDsl.() -> Unit): FollowPathCommand {
        MarkerDsl().apply(block).markers.forEach { a, b -> follower.addMarker(a, b) }
        return this
    }

    class MarkerDsl {
        val markers = mutableMapOf<Rectangle2d, () -> Unit>()

        operator fun Rectangle2d.plusAssign(callback: () -> Unit) {
            markers[this] = callback
        }
    }

    override suspend fun initialize() {
        LiveDashboard.isFollowingPath = true
        if (zeroPose) {
            Drivetrain.robotPosition = path.firstState.state.pose
        }
        Drivetrain.stop()
    }

    override suspend fun execute() {
        val currentPose = Drivetrain.robotPosition

        val state = follower.update(currentPose)
        val (left, right, laff, raff) = kinematicsHelper.update(state)
        Drivetrain.set(left, right, laff, raff)

        updateDashboard()
    }

    override suspend fun dispose() {
        Drivetrain.stop()
        LiveDashboard.isFollowingPath = false
    }

    private fun updateDashboard() {
        val seg = follower.referencePose

        LiveDashboard.pathX = seg.translation.x.feet
        LiveDashboard.pathY = seg.translation.y.feet
        LiveDashboard.pathHeading = seg.rotation.radian
    }
}