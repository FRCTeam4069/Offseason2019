package frc.team4069.robot.vision

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.SerialPort
import frc.team4069.robot.util.RingBuffer
import frc.team4069.saturn.lib.mathematics.units.*
import frc.team4069.saturn.lib.mathematics.units.conversions.inch
import frc.team4069.saturn.lib.util.launchFrequency
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.ObsoleteCoroutinesApi
import kotlinx.coroutines.delay
import kotlinx.coroutines.newSingleThreadContext
import kotlin.math.hypot

/**
 * Vision system coordinator for autonomous
 *
 * Handles data from the JeVois A33 camera, starts the video stream for dashboard and handles messages from serial
 */
object VisionSystem {
    // Variables containing the last value from the jevois
    // Null signifies no or invalid data from the last iteration of the loop
    internal val targetXs = RingBuffer(5)
    internal val targetZs = RingBuffer(5)

    /**
     * The distance to the vision target as determined by solvePnP on the jevois
     * The value may have sudden jumps above or below the actual value, so data is stored in a ring buffer
     * and median filtered for use in robot code
     */
    val targetDistance: SIUnit<Meter>?
        get() {
            val x = targetX ?: return null
            val z = targetZ ?: return null

            return hypot(x.value, z.value).meter
        }

    val targetX: SIUnit<Meter>?
        get() {
            return if(targetXs.numElements == 0) {
                null
            }else {
                targetXs.median.inch
            }
        }

    val targetZ: SIUnit<Meter>?
        get() {
            return if(targetZs.numElements == 0) {
                null
            }else {
                targetZs.median.inch
            }
        }

    init {
        val camera = CameraServer.getInstance().startAutomaticCapture()
        JevoisHandler // Start the jevois handler
    }

    fun markUnplugged() {
        targetXs.clear()
        targetZs.clear()
    }
}
