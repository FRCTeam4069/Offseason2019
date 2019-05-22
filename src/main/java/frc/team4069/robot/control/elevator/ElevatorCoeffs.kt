package frc.team4069.robot.control.elevator

import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceControllerCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceObserverCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpacePlantCoeffs
import koma.end
import koma.mat

object ElevatorCoeffs {
    val plantCoeffs =
            StateSpacePlantCoeffs(1, 2, 1,
                    A = mat[
                            1.0, 0.0038123917094548044 end
                            0.0, 0.09251846318419062
                    ],
                    B = mat[
                            0.000587130443210727, 0.08610920599650167
                    ].T,
                    C = mat[
                            1.0, 0.0
                    ],
                    D = mat[
                            0.0
                    ]
            )
    val controllerCoeffs =
            StateSpaceControllerCoeffs(1, 2,
                    K = mat[
                            197.74500975121012, 1.5868649265235129
                    ],
                    Kff = mat[
                            27.107309014237973, 9.9389704745734
                    ],
                    Umin = mat[-12.0],
                    Umax = mat[+12.0])
    val observerCoeffs =
            StateSpaceObserverCoeffs(2, 1,
                    K = mat[
                            0.9999757217645118, 0.6994463408695476
                    ].T
            )
}