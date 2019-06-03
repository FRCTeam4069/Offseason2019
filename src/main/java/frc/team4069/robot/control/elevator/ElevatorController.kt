package frc.team4069.robot.control.elevator

import edu.wpi.first.wpilibj.Timer
import frc.team4069.keigen.*
import frc.team4069.saturn.lib.mathematics.onedim.control.TrapezoidalProfile
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceController
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceObserver
import frc.team4069.saturn.lib.mathematics.statespace.StateSpacePlant
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.derivedunits.LinearVelocity
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.meter
import frc.team4069.saturn.lib.mathematics.units.second

class ElevatorController {
    private val plant = StateSpacePlant(ElevatorCoeffs.plantCoeffs)
    private val controller = StateSpaceController(ElevatorCoeffs.controllerCoeffs, plant)
    private val observer = StateSpaceObserver(ElevatorCoeffs.observerCoeffs, plant)

    var u = zeros(`1`)

    var reference = zeros(`2`)
    var y = zeros(`2`)

    val voltage get() = u[0, 0]
    val position get() = observer.xHat[0, 0].meter
    val velocity get() = observer.xHat[1, 0].meter.velocity

    var measuredPosition: Length
        get() = y[0].meter
        set(value) {
            y[0] = value.meter
        }

    var measuredVelocity: LinearVelocity
        get() = y[1].meter.velocity
        set(value) {
            y[1] = value.value
        }

    var motionProfile: TrapezoidalProfile? = null

    fun update() {
//        y[0, 0] = Elevator.position.meter

        if(motionProfile != null) {
            val data = motionProfile!!.getVelocity(Timer.getFPGATimestamp().second)
            reference = vec(`2`)
                    .fill(
                            data.x,
                            data.v
                    )
        }

//        observer.correct(u, y)
//        observer.predict(u)

        // Assume that y == x, kalman filter is breaking things
        controller.update(y, reference)
        this.u = controller.u
    }
}