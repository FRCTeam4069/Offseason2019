package frc.team4069.robot.control.elevator

import frc.team4069.keigen.*
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceControllerCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpaceObserverCoeffs
import frc.team4069.saturn.lib.mathematics.statespace.coeffs.StateSpacePlantCoeffs

object ElevatorCoeffs {
    val plantCoeffs =
            StateSpacePlantCoeffs(`2`, `1`, `1`,
                    A = mat[`2`, `2`,
                            1.0, 0.0038123917094548044,
                            0.0, 0.09251846318419062
                    ],
                    B = vec[`2`,
                            0.000587130443210727, 0.08610920599650167
                    ],
                    C = mat[`1`, `2`,
                            1.0, 0.0
                    ],
                    D = mat[`1`, `1`,
                            0.0
                    ]
            )
    val controllerCoeffs =
            StateSpaceControllerCoeffs(
                    K = mat[`1`, `2`,
                            197.74500975121012, 1.5868649265235129
                    ],
                    Kff = mat[`1`, `2`,
                            27.107309014237973, 9.9389704745734
                    ],
                    Umin = vec[`1`, -12.0],
                    Umax = vec[`1`, +12.0])
    val observerCoeffs =
            StateSpaceObserverCoeffs(
                    K = vec[`2`,
                            0.9999757217645118, 0.6994463408695476
                    ]
            )
}