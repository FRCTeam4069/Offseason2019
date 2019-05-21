package frc.team4069.robot.commands.drive

import frc.team4069.robot.subsystems.Drivetrain
import frc.team4069.robot.vision.VisionSystem
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.debug.LiveDashboard
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity

//class NavigateToTapePID(val averageWheelSpeed: LinearVelocity) : SaturnCommand(Drivetrain) {
//
//    var lastAngle = 0.0
//    var lastTime = 0.0
//
//    override suspend fun execute() {
//        val currentTime = System.currentTimeMillis() / 1000.0
//        val deltaTime = currentTime - lastTime
//        lastTime = currentTime
//
//        if (VisionSystem.anglePair == null) {
//            return
//        }
//
//        val meanAngle = (VisionSystem.anglePair!!.first.degree + VisionSystem.anglePair!!.second.degree) / 2
//        val derivativeAngle = (meanAngle - lastAngle) / deltaTime
//        lastAngle = meanAngle
//
//        val rightWheelSpeedOverAverage = 1.0 + (meanAngle / 45.0) + (derivativeAngle * -0.5)
//        val rightWheelSpeed = averageWheelSpeed * rightWheelSpeedOverAverage
//        val leftWheelSpeed = averageWheelSpeed * (2.0 - rightWheelSpeedOverAverage)
//        Drivetrain.set(leftWheelSpeed, rightWheelSpeed)
//    }
//
//    override suspend fun dispose() {
//        Drivetrain.stop()
//        LiveDashboard.isFollowingPath = false
//    }
//}