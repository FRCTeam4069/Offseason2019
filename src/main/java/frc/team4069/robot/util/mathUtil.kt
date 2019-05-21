package frc.team4069.robot.util

import frc.team4069.saturn.lib.mathematics.twodim.geometry.Pose2d

// Rounds the number i to the closest multiple of j
fun round(i: Double, j: Double) = kotlin.math.round(i / j) * j

private val FIELD_ORIGIN = Pose2d()

/**
 * Transform a robot-relative Pose2d into a field-relative Pose2d
 */
fun Pose2d.toFieldReference(): Pose2d {
    return this inFrameOfReferenceOf FIELD_ORIGIN
}
