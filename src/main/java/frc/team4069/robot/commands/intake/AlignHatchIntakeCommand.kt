package frc.team4069.robot.commands.intake

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.robot.subsystems.intake.SlideIntake
import frc.team4069.robot.vision.VisionSystem
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.conversions.degree
import frc.team4069.saturn.lib.mathematics.units.conversions.inch

class AlignHatchIntakeCommand : SaturnCommand(SlideIntake) {
    override fun execute() {
        if(OI.sliderOperatorControl) {
            SlideIntake.setDutyCycle(OI.slideSpeed)
        }else {
            // Intake is extended, both in code and according to the sensor, and the pivot is close to idle
            if (Intake.extended && Intake.angle.degree < 50.0 && Intake.pivot.selectedSensorVelocity < 50) {
                val dist = (VisionSystem.targetX ?: return)
                val transformedDist = dist + 6.inch // Transform into slideshift f.o.r

                // Make sure that the target is in the min..max of the slider, then go there
                if (transformedDist.inch in 0.0..11.0) {
                    SlideIntake.setPosition(transformedDist)
                }
            }
        }
    }
}