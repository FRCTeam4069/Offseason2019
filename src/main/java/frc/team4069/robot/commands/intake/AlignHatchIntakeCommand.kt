package frc.team4069.robot.commands.intake

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.intake.Intake
import frc.team4069.robot.subsystems.intake.SlideIntake
import frc.team4069.robot.vision.VisionSystem
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.inch

class AlignHatchIntakeCommand : SaturnCommand(SlideIntake) {
    override suspend fun execute() {
        if(OI.sliderOperatorControl) {
            SlideIntake.setDutyCycle(OI.slideSpeed)
        }else {
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