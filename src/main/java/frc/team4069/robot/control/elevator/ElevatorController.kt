package frc.team4069.robot.control.elevator

import frc.team4069.keigen.*
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceController
import frc.team4069.saturn.lib.mathematics.statespace.StateSpaceObserver
import frc.team4069.saturn.lib.mathematics.statespace.StateSpacePlant
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.derivedunits.velocity
import frc.team4069.saturn.lib.mathematics.units.meter

class ElevatorController {
    private val plant = StateSpacePlant(ElevatorCoeffs.plantCoeffs)
    private val controller = StateSpaceController(ElevatorCoeffs.controllerCoeffs, plant)
    private val observer = StateSpaceObserver(ElevatorCoeffs.observerCoeffs, plant)

    var u = zeros(`1`)

    var reference = zeros(`2`)
    var y = zeros(`1`)

    val voltage get() = u[0, 0]
    val position get() = observer.xHat[0, 0].meter
    val velocity get() = observer.xHat[1, 0].meter.velocity

    var measuredPosition: Length
        get() = y[0, 0].meter
        set(value) {
            y[0, 0] = value.meter
        }

    fun update() {
//        y[0, 0] = Elevator.position.meter
        observer.correct(u, y)
        observer.predict(u)

        controller.update(observer.xHat, reference)
        this.u = controller.u
    }
}