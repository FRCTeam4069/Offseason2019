package frc.team4069.robot.commands.elevator

import frc.team4069.robot.OI
import frc.team4069.robot.subsystems.Elevator
import frc.team4069.saturn.lib.commands.SaturnCommand

class OperatorElevatorCommand : SaturnCommand(Elevator) {

    var set = false

    override fun execute() {
        val spd = OI.elevatorSpeed // Deadbanded in OI

        if(spd != 0.0) {
            Elevator.setDutyCycle(spd)
            set = false
        }else {
            if(!set) {
//                 Hold the elevator in place once operator lets off the stick
                Elevator.setPosition(Elevator.position)
                set = true
            }
        }
    }
}