package org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces;

public interface Enums {

    enum Hubs{
        CONTROL_HUB,
        EXPANSION_HUB,
        ALL
    }

    enum Pipelines{
        DETECTING_PROP
    }

    enum Randomization{
        LEFT,
        CENTER,
        RIGHT
    }

    enum OpMode{
        TELE_OP,
        AUTONOMUS
    }

    enum Gamepads{
        G1, G2,
        BOTH
    }

    enum Rumbles{ }

    interface Pathing{
        enum Polynomial{
            UNDEFINED, constant, linear, quadratic, cubic, quartic, quintic, MULTIPLE
        }
    }

    interface Mecanum {
        enum MotionPackage {
            ROADRUNNER,
            CUSTOM,
            PID
        }

        enum Localizers {
            CUSTOM,
            IMU,
            ROADRUNNER_THREE_WHEELS,
            ROADRUNNER_TWO_WHEELS
        }
    }
}
