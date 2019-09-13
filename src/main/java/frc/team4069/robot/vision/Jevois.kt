package frc.team4069.robot.vision

import com.fazecast.jSerialComm.SerialPort
import com.fazecast.jSerialComm.SerialPortDataListener
import com.fazecast.jSerialComm.SerialPortEvent
import edu.wpi.first.wpilibj.Timer
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.Second
import frc.team4069.saturn.lib.mathematics.units.hertz
import frc.team4069.saturn.lib.mathematics.units.second
import frc.team4069.saturn.lib.util.launchFrequency
import kotlinx.coroutines.GlobalScope
import kotlinx.serialization.Serializable
import kotlinx.serialization.cbor.Cbor
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonConfiguration

object JevoisHandler {
    private val connectedCameras = mutableListOf<Jevois>()

    init {
        GlobalScope.launchFrequency(0.1.hertz) {
            val currentTime = Timer.getFPGATimestamp().second

            connectedCameras.removeIf {
                it.update(currentTime)
                if(!it.isAlive) {
                    it.dispose()
                    true
                }else {
                    false
                }
            }

            val ports = SerialPort.getCommPorts()
                    .filter { it.descriptivePortName.contains("JeVois", true) }

            for(port in ports) {
                if(connectedCameras.any { it.systemPortName == port.systemPortName })
                    continue

                connectedCameras.add(Jevois(port))
            }
        }
    }
}

class Jevois(private val serialPort: SerialPort) : SerialPortDataListener{

    private val codec = Json(JsonConfiguration.Stable)

    val systemPortName: String = serialPort.descriptivePortName
    private var wasUnplugged = false
    private var lastTimeReceived = 0.second

    var isAlive = true
        private set

    @ExperimentalStdlibApi
    override fun serialEvent(e: SerialPortEvent) {
        if(e.eventType != SerialPort.LISTENING_EVENT_DATA_AVAILABLE)
            return

        val bytesAvailable = serialPort.bytesAvailable()
        if(bytesAvailable < 0) {
            wasUnplugged = true
            return
        }

        val data = ByteArray(bytesAvailable)
        serialPort.readBytes(data, bytesAvailable.toLong())

        processMessage(codec.parse(JevoisData.serializer(), data.decodeToString()))
    }

    override fun getListeningEvents(): Int = SerialPort.LISTENING_EVENT_DATA_AVAILABLE

    private fun processMessage(data: JevoisData) {
        lastTimeReceived = Timer.getFPGATimestamp().second
        VisionSystem.targetXs.add(data.targetX)
        VisionSystem.targetZs.add(data.targetZ)
    }

    fun postMessage(command: PostCommand) {
        val data = codec.toJson(PostCommand.serializer(), command).toString()
        serialPort.writeBytes(data.toByteArray(), data.length.toLong())
    }

    fun update(currentTime: SIUnit<Second>) {
        isAlive = !wasUnplugged && currentTime - lastTimeReceived <= 1.second
    }

    fun dispose() {
        VisionSystem.markUnplugged()
        serialPort.closePort()
    }

    init {
        serialPort.openPort()
        serialPort.addDataListener(this)
    }
}

@Serializable
data class JevoisData(val time: Long,
                      val targetX: Double,
                      val targetZ: Double)

@Serializable
data class PostCommand(val time: Long,
                       val code: String)


