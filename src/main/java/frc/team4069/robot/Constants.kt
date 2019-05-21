package frc.team4069.robot

import frc.team4069.saturn.lib.mathematics.units.feet
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import frc.team4069.saturn.lib.mathematics.units.nativeunits.STU

object Constants {
    val DRIVETRAIN_WHEELBASE = 3.9.feet // Empirically determined by spinning fast in a circle
    // All FF constants are metric. Determined from robotpy/robot-characterization
    // kV is sm^-1. kA is s^2m^-1. kS is %vbus
    const val DT_LEFT_KV = 0.287
    const val DT_RIGHT_KV = 0.282

    const val DT_LEFT_KA = 0.02
    const val DT_RIGHT_KA = 0.027

    const val DT_LEFT_KS = 0.091
    const val DT_RIGHT_KS = 0.089
    // Ramsete constants
    const val kZeta = 0.99
    const val kB = 3.3 //3.2

    // Native unit models, translates from STU into real world units
    val DT_MODEL = NativeUnitLengthModel(4096.STU, 3.89.inch)
    val ELEVATOR_MODEL = NativeUnitLengthModel(4096.STU, 0.91.inch)
    val INTAKE_SLIDE_MODEL = NativeUnitLengthModel(4096.STU, 0.56.inch)
}