package frc.team4069.robot.commands.elevator

import frc.team4069.robot.subsystems.Elevator
import frc.team4069.saturn.lib.commands.SaturnCommand
import frc.team4069.saturn.lib.mathematics.units.Meter
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import kotlin.math.abs

class SetElevatorPositionCommand(private val position: SIUnit<Meter>, private val instant: Boolean = false) : SaturnCommand(Elevator) {

    constructor(pos: Elevator.Position, instant: Boolean = false) : this(pos.length, instant)

    override fun initialize() {
        Elevator.setPosition(position)
    }

    override fun isFinished(): Boolean = instant || abs((Elevator.position - position).inch) < 0.8
}