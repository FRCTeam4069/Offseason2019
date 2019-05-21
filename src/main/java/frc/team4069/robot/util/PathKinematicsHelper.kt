package frc.team4069.robot.util

import com.team254.lib.physics.DifferentialDrive
import frc.team4069.robot.Constants
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.feet
import frc.team4069.saturn.lib.mathematics.units.millisecond
import frc.team4069.saturn.lib.util.DeltaTime
import kotlin.math.sign

/**
 * Class encapsulating shared logic of all trajectory-following commands
 *
 * Once a ChassisState has been obtained through the follower, call PathKinematicsHelper#update() to get the
 * required left and right velocities, as well as arbitrary feedforwards to attain the state
 */
class PathKinematicsHelper {
    private var lastState: DifferentialDrive.WheelState? = null
    private val dtController = DeltaTime()

    fun update(state: DifferentialDrive.ChassisState): MotorCommands {
        val wheelState = inverseKinematics(state, Constants.DRIVETRAIN_WHEELBASE)

        // AFF is only kS, we add kA if there has been a state in the path such that we can differentiate velocities
        // kV is handled through the talon's kF to allow for motion magic
        var leftAff = Constants.DT_LEFT_KS * sign(wheelState.left)
        var rightAff = Constants.DT_RIGHT_KS * sign(wheelState.right)

        if(lastState != null) {
            val dt = dtController.updateTime(System.currentTimeMillis().millisecond).second
            val accelLeft = (wheelState.left - lastState!!.left) / dt
            val accelRight = (wheelState.right - lastState!!.right) / dt

            leftAff += Constants.DT_LEFT_KA * accelLeft
            rightAff += Constants.DT_RIGHT_KA * accelRight
        }
        lastState = wheelState

        return MotorCommands(
                wheelState.left.feet.velocity,
                wheelState.right.feet.velocity,
                leftAff,
                rightAff
        )
    }

    private fun inverseKinematics(state: DifferentialDrive.ChassisState,
                          wheelBase: Length
    ): DifferentialDrive.WheelState {
        val left = ((-wheelBase.meter * state.angular + 2 * state.linear) / 2)
        val right = ((wheelBase.meter * state.angular + 2 * state.linear) / 2)

        return DifferentialDrive.WheelState(left, right)
    }

    data class MotorCommands(
            val leftVel: LinearVelocity,
            val rightVel: LinearVelocity,
            val leftAff: Double,
            val rightAff: Double
    )
}