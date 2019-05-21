package frc.team4069.robot.util

// Class that manages current limiting while acting as an Evicting Queue
// Thanks 5190 for the algo
class RingBuffer(private val size: Int) {

    private val buffer: ArrayList<Double> = ArrayList(size)

    var numElements = 0
    private var sum = 0.0

    // Gets average of all elements
    val average: Double
        get() {
            return if (numElements == 0)
                0.0
            else
                sum / numElements
        }

    val median: Double
        get() {
            val buf = buffer
            return if(numElements == 0) {
                0.0
            }else {
                val sorted = buf.sorted()
                val size = sorted.size
                when {

                    size % 2 == 0 -> {
                        (sorted[size/2] + sorted[(size/2)-1])/2.0
                    }
                    else -> sorted[(size-1)/2]
                }
            }
        }

    // Adds an element to the list
    fun add(element: Double) {
        if (numElements > size - 1) {
            sum -= buffer[size - 1]
            buffer.removeAt(size - 1)
            numElements--
        }
        sum += element
        buffer.add(0, element)
        numElements++
    }

    fun clear() {
        buffer.clear()
        sum = 0.0
        numElements = 0
    }
}