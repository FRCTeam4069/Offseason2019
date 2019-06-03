package frc.team4069.robot.control.elevator

import frc.team4069.keigen.*
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceControllerCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceObserverCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpacePlantCoeffs

object ElevatorCoeffs {
    val plantCoeffs =
            StateSpacePlantCoeffs(`2`, `1`, `2`,
                    A = mat(`2`, `2`).fill(
                            1.0, 0.000672170658888835,
                            0.0, 3.458816828536931e-07
                    ),
                    B = vec(`2`).fill(
                            0.0003540400308538644, 0.037955230038088185
                    ),
                    C = eye(`2`),
                    D = zeros(`2`, `1`)
            )
    val controllerCoeffs =
            StateSpaceControllerCoeffs(
                    K = mat(`1`, `2`).fill(
                            197.74500975121012, 1.5868649265235129
                    ),
                    Kff = mat(`1`, `2`).fill(
                            27.107309014237973, 9.9389704745734
                    ),
                    Umin = vec(`1`).fill(-12.0),
                    Umax = vec(`1`).fill(+12.0)
            )
    val observerCoeffs =
            StateSpaceObserverCoeffs(
                    K = mat(`2`, `2`).fill(
                            0.3279215611246275, 6.247005370670148e-14,
                            6.247005370671573e-18, 0.9996001599360256
                    )
            )
}