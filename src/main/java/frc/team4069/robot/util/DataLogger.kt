package frc.team4069.robot.util

import krangl.*
import java.io.File

object DataLogger {
//    val stateFrame = dataFrameOf("Time", "Measured Position", "Measured Velocity")()
//    val refFrame = dataFrameOf("Time", "Position Reference", "Velocity Reference")()
//    val voltageFrame = dataFrameOf("Time", "Output Voltage", "DS Voltage")()
    val states = mutableListOf<StateEntry>()
    val refs = mutableListOf<RefEntry>()
    val volts = mutableListOf<VoltageEntry>()

    var enabled = false

    data class StateEntry(val time: Double, val pos: Double, val vel: Double)

    data class RefEntry(val time: Double, val posRef: Double, val velRef: Double)

    data class VoltageEntry(val time: Double, val outVolts: Double, val dsVolts: Double)

    fun append(state: StateEntry, ref: RefEntry, voltage: VoltageEntry) {
        if(enabled) {
            states.add(state)
            refs.add(ref)
            volts.add(voltage)
        }
    }

    fun writeToCSVs() {
        val stateFile = File("/home/lvuser/States.csv")
        stateFile.createNewFile()
        val stateFrame = DataFrame.fromRecords(states) {
            mapOf(
                    "Time" to it.time,
                    "Measured Position" to it.pos,
                    "Measured Velocity" to it.vel
            )
        }
        stateFrame.writeCSV(stateFile)


        val refFile = File("/home/lvuser/References.csv")
        refFile.createNewFile()

        val refFrame = DataFrame.fromRecords(refs) {
            mapOf(
                    "Time" to it.time,
                    "Position Reference" to it.posRef,
                    "Velocity Reference" to it.velRef
            )
        }
        refFrame.writeCSV(refFile)

        val voltFile = File("/home/lvuser/Voltages.csv")
        voltFile.createNewFile()
        val voltFrame = DataFrame.fromRecords(volts) {
            mapOf(
                    "Time" to it.time,
                    "Output Voltage" to it.outVolts,
                    "DS Voltage" to it.dsVolts
            )
        }
        voltFrame.writeCSV(voltFile)
    }
}

