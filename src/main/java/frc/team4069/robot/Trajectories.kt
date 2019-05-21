package frc.team4069.robot

import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.util.toFieldReference
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Rectangle2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Translation2d
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.constraints.TimingConstraint
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint
import frc.team4069.saturn.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearAcceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.acceleration
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity

/**
 * Contains all pre-generated trajectories for use in autonomous
 * Also contains logic to generate trajectories on the fly to correct for deviations from expected behaviour in precision-critical maneuvers
 */
object Trajectories {
    private val kMaxVelocity = 2.feet.velocity
    private val kMaxAcceleration = 2.feet.acceleration
    private val kMaxCentripetalAcceleration = 4.feet.acceleration

    private val kConstraints = listOf(
            CentripetalAccelerationConstraint(kMaxCentripetalAcceleration)
    )

    val cameraToIntake = Pose2d(-25.inch, 0.inch)

    val frontCargoShip = waypoints(
            Pose2d(5.498.feet, 13.71.feet, 0.degree),
            Pose2d(13.636.feet, 12.542.feet, 0.degree)
    ).generateTrajectory(
            "Front Cargo Ship",
            reversed = false,
            maxVelocity = 7.feet.velocity,
            maxAcceleration = 4.feet.acceleration,
            constraints = listOf(
                    VelocityLimitRegionConstraint(
                            Rectangle2d(Translation2d(0.feet, 0.feet),
                                    Translation2d(8.5.feet, 27.feet)),
                            1.feet.velocity
                    ),
                    CentripetalAccelerationConstraint(3.feet.acceleration)
            )
    )

    val backRightRocket = waypoints(
            Pose2d(5.498.feet, 9.405.feet, 180.degree),
            Pose2d(16.093.feet, 7.792.feet, 158.degree),
            Pose2d(24.258.feet, 7.44.feet, -98.degree)
    ).generateTrajectory(
            "Rocket Right (Backwards)",
            reversed = true,
            maxVelocity = 6.5.feet.velocity,
            maxAcceleration = 4.feet.acceleration,
            constraints = listOf(
                    // Off the platform
                    VelocityLimitRegionConstraint(
                            Rectangle2d(Translation2d(0.feet, 0.feet),
                                    Translation2d(8.5.feet, 27.feet)),
                            3.5.feet.velocity
                    ),
                    CentripetalAccelerationConstraint(2.feet.acceleration)
            )
    )

    val rocketRightOffset = waypoints(
            Pose2d(5.498.feet, 9.405.feet, 0.degree),
            Pose2d(12.392.feet, 5.873.feet, -50.degree),
            Pose2d(16.32.feet, 2.386.feet, -30.degree)
    ).generateTrajectory(
            "Rocket Right Offset",
            reversed = false,
            maxVelocity = 5.feet.velocity,
            maxAcceleration = 4.feet.acceleration,
            constraints = listOf(
                    // Off the platform
                    VelocityLimitRegionConstraint(
                            Rectangle2d(Translation2d(0.feet, 0.feet),
                                    Translation2d(8.5.feet, 27.feet)),
                            2.5.feet.velocity
                    ),
                    // Honing in on the target
                    VelocityLimitRegionConstraint(
                            Rectangle2d(Translation2d(13.feet, 2.feet),
                                    Translation2d(18.feet, 5.feet)),
                            1.feet.velocity
                    ),
                    CentripetalAccelerationConstraint(2.feet.acceleration)
            )
    )

    // using mirror directly was sketchy and slow
    val rocketLeftOffset = waypoints(
            Pose2d(5.498.feet, 27.feet - 9.405.feet, 0.degree),
            Pose2d(12.392.feet, 27.feet - 5.873.feet, 50.degree),
            Pose2d(16.32.feet, 27.feet - 2.386.feet, 30.degree)
    )
            .generateTrajectory(
                    "Rocket Left Offset",
                    reversed = false,
                    maxVelocity = 5.feet.velocity,
                    maxAcceleration = 4.feet.acceleration,
                    constraints = listOf(
                            // Off the platform
                            VelocityLimitRegionConstraint(
                                    Rectangle2d(Translation2d(0.feet, 0.feet),
                                            Translation2d(8.5.feet, 27.feet)),
                                    2.5.feet.velocity
                            ),
                            // Honing in on the target
                            // 27 - 2 = 25
                            // 27 - 5 = 22
                            VelocityLimitRegionConstraint(
                                    Rectangle2d(Translation2d(13.feet, 22.feet),
                                            Translation2d(18.feet, 25.feet)),
                                    1.feet.velocity
                            ),
                            CentripetalAccelerationConstraint(2.feet.acceleration)
                    )
            )

    val rocketRightToHalfHP = waypoints(
            Pose2d(16.32.feet, 2.386.feet, -34.degree),
            Pose2d(13.953.feet, 4.423.feet, -132.degree)
    ).generateTrajectory(
            "Rocket Halfway to HP",
            reversed = true,
            maxVelocity = 4.feet.velocity,
            maxAcceleration = 2.feet.acceleration
    )

    val rocketLeftToHalfHP = waypoints(
            Pose2d(16.32.feet, 27.feet - 2.386.feet, 34.degree),
            Pose2d(13.953.feet, 27.feet - 4.423.feet, 132.degree)
    ).generateTrajectory(
            "Rocket Halfway to HP (Left)",
            reversed = true,
            maxVelocity = 4.feet.velocity,
            maxAcceleration = 2.feet.acceleration
    )

    val backRocketToTarget = waypoints(
            Pose2d(24.258.feet, 7.44.feet, -98.degree),
            Pose2d(22.293.feet, 2.106.feet, -150.degree)
    ).generateTrajectory(
            "Back Rocket Mark To Target",
            reversed = false,
            maxVelocity = 1.5.feet.velocity,
            maxAcceleration = 0.75.feet.acceleration
    )

    val reverseBackRocketToTarget = waypoints(
            Pose2d(22.293.feet, 2.106.feet, -150.degree),
            Pose2d(24.258.feet, 7.44.feet, -98.degree)
    ).generateTrajectory(
            "Back Rocket Mark To Target (Reversed)",
            reversed = true,
            maxVelocity = 1.5.feet.velocity,
            maxAcceleration = 1.5.feet.acceleration
    )

    val backMarkToHP = waypoints(
            Pose2d(24.258.feet, 7.44.feet, -98.degree),
            Pose2d(17.611.feet, 5.662.feet, -170.degree),
            Pose2d(1.694.feet, 2.059.feet, 180.degree)
    ).generateTrajectory(
            "Back Rocket To Human Player",
            reversed = false,
            maxVelocity = 7.feet.velocity,
            maxAcceleration = 5.feet.acceleration,
            constraints = listOf(
                    CentripetalAccelerationConstraint(2.feet.acceleration),
                    VelocityLimitRegionConstraint(
                            Rectangle2d(
                                    Translation2d(0.feet, 0.feet),
                                    Translation2d(5.feet, 5.feet)
                            ),
                            1.25.feet.velocity
                    )
            )
    )

    val cargoShipRightOffset = waypoints(
            Pose2d(5.498.feet, 9.405.feet, 0.degree),
            Pose2d(13.907.feet, 8.073.feet, -48.degree),
//            Pose2d(21.448.feet, 9.437.feet, 90.degree)
            Pose2d(21.448.feet, 7.955.feet, 90.degree)
    ).generateTrajectory(
            "Cargo Ship Right Offset",
            reversed = false,
            maxVelocity = 5.feet.velocity,
            maxAcceleration = 4.feet.acceleration,
            constraints = listOf(
                    VelocityLimitRegionConstraint(
                            Rectangle2d(Translation2d(0.feet, 0.feet),
                                    Translation2d(8.5.feet, 27.feet)),
                            3.feet.velocity
                    ),
//                    VelocityLimitRegionConstraint(
//                            Rectangle2d(
//                                    Translation2d(13.feet, 4.feet),
//                                    Translation2d(20.feet, 9.feet)
//                            ),
//                            1.5.feet.velocity
//                    ),
                    VelocityLimitRegionConstraint(
                            Rectangle2d(
                                    Translation2d(14.5.feet, 4.feet),
                                    Translation2d(20.feet, 5.feet)
                            ),
                            2.feet.velocity
                    ),
                    VelocityLimitRegionConstraint(
                            Rectangle2d(Translation2d(20.feet, 5.feet),
                                    Translation2d(24.feet, 12.feet)),
                            1.feet.velocity
                    ),
                    CentripetalAccelerationConstraint(2.feet.acceleration)
            )
    )

    val rightHalfToHP = waypoints(
            Pose2d(13.953.feet, 4.423.feet, -132.degree),
            Pose2d(1.694.feet, 2.059.feet, 180.degree)
    ).generateTrajectory(
            "Halfway Mark to HP",
            reversed = false,
            maxVelocity = 7.feet.velocity,
            maxAcceleration = 5.feet.acceleration,
            constraints = listOf(
                    VelocityLimitRegionConstraint(
                            Rectangle2d(
                                    Translation2d(0.feet, 0.feet),
                                    Translation2d(5.feet, 5.feet)
                            ),
                            1.25.feet.velocity
                    ),
                    CentripetalAccelerationConstraint(3.5.feet.acceleration)
            )
    )

    val leftHalfToHP = waypoints(
            Pose2d(13.953.feet, 27.feet - 4.423.feet, 132.degree),
            Pose2d(1.694.feet, 27.feet - 2.059.feet, 180.degree)
    ).generateTrajectory(
            "Halfway Mark to HP (Left",
            reversed = false,
            maxVelocity = 7.feet.velocity,
            maxAcceleration = 5.feet.acceleration,
            constraints = listOf(
                    VelocityLimitRegionConstraint(
                            Rectangle2d(
                                    Translation2d(0.feet, 22.feet),
                                    Translation2d(5.feet, 27.feet)
                            ),
                            1.25.feet.velocity
                    ),
                    CentripetalAccelerationConstraint(3.5.feet.acceleration)
            )
    )

    fun trapezoidToDist(startPose: Pose2d = Drivetrain.robotPosition,
                        dist: Length,
                        overrideEndRot: Rotation2d? = null,
                        maxVelocity: LinearVelocity = kMaxVelocity,
                        startVelocity: LinearVelocity = 0.feet.velocity,
                        constraints: List<TimingConstraint<Pose2dWithCurvature>> = kConstraints): TimedTrajectory<Pose2dWithCurvature> {
        // The robot-relative rotation if this isn't null will be the difference between it and the current angle
        val rot = if (overrideEndRot != null) {
            overrideEndRot - startPose.rotation
        } else {
            0.radian
        }

        // Transform the robot-relative pose to be field relative
        val distPose = Pose2d(dist, 0.feet, rot).toFieldReference()
        return waypoints(startPose, startPose + distPose)
                .generateTrajectory("Dynamic Trapezoidal Profile",
                        reversed = false,
                        maxVelocity = maxVelocity,
                        startVelocity = startVelocity,
                        constraints = constraints
                )
    }

    fun navigateToRelativePosition(
            startPose: Pose2d = Drivetrain.robotPosition,
            relativeEndingPose: Pose2d,
            constraints: List<TimingConstraint<Pose2dWithCurvature>> = kConstraints,
            midpoints: List<Pose2d>? = null,
            maxVelocity: LinearVelocity = kMaxVelocity,
            startVelocity: LinearVelocity = 0.feet.velocity,
            reversed: Boolean
    ): TimedTrajectory<Pose2dWithCurvature> {
        return if (midpoints == null) {
            waypoints(startPose, startPose + relativeEndingPose)
                    .generateTrajectory("Trajectory to Relative Position", reversed = reversed, maxVelocity = maxVelocity,
                            constraints = constraints,
                            startVelocity = startVelocity)
        } else {
            waypoints(startPose, *midpoints.map { startPose + it }.toTypedArray(), startPose + relativeEndingPose)
                    .generateTrajectory("Trajectory to Relative Position", reversed = reversed, maxVelocity = maxVelocity,
                            constraints = constraints,
                            startVelocity = startVelocity)
        }
    }


    private fun waypoints(vararg waypoints: Pose2d) = waypoints.toList()

    private fun List<Pose2d>.generateTrajectory(
            name: String,
            reversed: Boolean,
            startVelocity: LinearVelocity = 0.meter.velocity,
            maxVelocity: LinearVelocity = kMaxVelocity,
            maxAcceleration: LinearAcceleration = kMaxAcceleration,
            constraints: List<TimingConstraint<Pose2dWithCurvature>> = kConstraints
    ): TimedTrajectory<Pose2dWithCurvature> {
        println("Generating $name")
        return DefaultTrajectoryGenerator.generateTrajectory(
                reversed = reversed,
                wayPoints = this,
                constraints = constraints,
                startVelocity = startVelocity,
                endVelocity = 0.meter.velocity,
                maxVelocity = maxVelocity,
                maxAcceleration = maxAcceleration
        )
    }
}