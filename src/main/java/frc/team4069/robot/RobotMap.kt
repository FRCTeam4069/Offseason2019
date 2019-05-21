package frc.team4069.robot

/**
 * Contains mappings of robot motor controllers and solenoids to their respective system
 */
object RobotMap {
    object Drivetrain {
        const val LEFT_MAIN_SRX = 9
        const val LEFT_SLAVE_SRX = 8

        const val RIGHT_MAIN_SRX = 6
        const val RIGHT_SLAVE_SRX = 7

        const val AUXILIARY_MAIN_SRX = 12
        const val AUXILIARY_SLAVE_SRX = 13
    }

    object Climber {
        const val MAIN_MAX = 14
        const val SLAVE_MAX = 15

        const val SOLENOID_FWD = 2
        const val SOLENOID_BCK = 5
    }

    object Elevator {
        const val MAIN_SRX = 4
        const val SLAVE_SRX = 5
    }

    object Intake {
        const val MAIN_SRX = 11

        const val PIVOT_SRX = 2
        const val SLIDE_SRX = 10
    }
}
