package frc.team4069.robot.util

import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController

/**
 * Class encapsulating a REV Analog Pressure Sensor (REV-11-1107) connected to the RIO over analog in
 */
class PressureSensor(port: Int) {
    private val ain = AnalogInput(port)

    /**
     * Gets the pressure measured from the sensor
     *
     * Returns a value in PSI
     */
    val pressure: Double
        get() {
            val controller5V = RobotController.getVoltage5V()
            return 250.0 * (ain.voltage / controller5V) - 25.0
        }
}