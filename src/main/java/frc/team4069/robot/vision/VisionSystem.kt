package frc.team4069.robot.vision

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.SerialPort
import frc.team4069.robot.util.RingBuffer
import frc.team4069.saturn.lib.mathematics.units.Length
import frc.team4069.saturn.lib.mathematics.units.derivedunits.hertz
import frc.team4069.saturn.lib.mathematics.units.inch
import frc.team4069.saturn.lib.mathematics.units.radian
import frc.team4069.saturn.lib.util.launchFrequency
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.ObsoleteCoroutinesApi
import kotlinx.coroutines.delay
import kotlinx.coroutines.newSingleThreadContext

/**
 * Vision system coordinator for autonomous
 *
 * Handles data from the JeVois A33 camera, starts the video stream for dashboard and handles messages from serial
 */
@ObsoleteCoroutinesApi
object VisionSystem {
    private val visionCtx = CoroutineScope(newSingleThreadContext("Vision Thread"))

    // Angle data provided over serial from Jevois
    private val port = SerialPort(115200, SerialPort.Port.kUSB, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne)

    // Variables containing the last value from the jevois
    // Null signifies no or invalid data from the last iteration of the loop
//    var anglePair: Pair<Rotation2d, Rotation2d>? = null
    private var targetDistances = RingBuffer(10)
    private var targetXs = RingBuffer(5)
    private val targetZs = RingBuffer(5)


    val CENTRE_OFFSET = -0.01.radian


    /**
     * The distance to the vision target as determined by solvePnP on the jevois
     * The value may have sudden jumps above or below the actual value, so data is stored in a ring buffer
     * and median filtered for use in robot code
     */
    val targetDistance: Length?
        get() {
            val med = targetDistances.median
            return if (med <= 0.0) {
                null
            } else {
                med.inch
            }
        }

    val targetX: Length?
        get() {
            return if(targetXs.numElements == 0) {
                null
            }else {
                targetXs.median.inch
            }
        }

    val targetZ: Length?
        get() {
            return if(targetZs.numElements == 0) {
                null
            }else {
                targetZs.median.inch
            }
        }

    init {
        val camera = CameraServer.getInstance().startAutomaticCapture()

        var count = 0
        visionCtx.launchFrequency(100.hertz) {
            if (port.bytesReceived == 0) {
                count++
                return@launchFrequency
            }

            val msg = port.readLine().trim()

            if (msg.isNotEmpty()) {
                if (msg.startsWith("RDIST")) {
                    val rest = msg.substring(5).trim().split(",")
                    val d = rest[0].toDouble()
                    val x = rest[1].toDouble()
                    val z = rest[2].toDouble()
                    targetDistances.add(d)
                    targetXs.add(x)
                    targetZs.add(z)

                    count = 0
                }
            } else {
                // If there have been multiple iterations where there was no data from solvePnP, the buffer should be cleared
                // So that stale data doesn't affect future use
                if (count > 5) {
                    targetDistances.clear()
                    targetXs.clear()
                    targetZs.clear()
                }
            }
        }
    }
}

/**
 * Reads a string from the given [SerialPort], 1 byte at a time up until a line feed ('\n')
 */
private suspend fun SerialPort.readLine(): String {
    val buf = StringBuilder()

    while (true) {
        val next = read(1).firstOrNull()?.toChar()
        if (next != null) {
            if (next == '\n') {
                return buf.toString()
            }

            buf.append(next)
        } else {
            delay(1)
        }
    }
}
