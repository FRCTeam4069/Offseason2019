package frc.team4069.robot.commands.elevator

import frc.team4069.robot.subsystems.Elevator
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.Length
import kotlin.math.abs

class SetElevatorPositionCommand(private val position: Length, instant: Boolean = false) : SaturnCommand(Elevator) {

    constructor(pos: Elevator.Position, instant: Boolean = false) : this(pos.length, instant)

    init {
        finishCondition += {
            abs((Elevator.position - position).inch) < 0.8
        }

        finishCondition += { instant }
    }

    override suspend fun initialize() {
        Elevator.setPosition(position)
    }
}