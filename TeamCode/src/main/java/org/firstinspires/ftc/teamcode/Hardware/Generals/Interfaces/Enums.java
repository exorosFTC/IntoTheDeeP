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

    enum Telemetry{
        DASHBOARD,
        REGULAR
    }

    enum Gamepads{
        G1, G2,
        BOTH
    }

    enum Rumbles{ }



    enum IntakeAction{
        INIT,
        SPIT,
        STOP,
        COLLECT,
        TRANSFER,
        DISABLE
    }

    enum Color{
        RED,
        BLUE,
        NONE
    }


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

    interface Outtake {
        enum ArmAction{
            TRANSFER,
            DISABLE,
            COLLECT,
            SCORE,
            HANG,
            INIT
        }

        enum LiftAction{
            ZERO(0),
            FULL(0),

            COLLECT(0),

            HIGH_BASKET(0),
            HIGH_RUNG(0),
            LOW_BASKET(0),
            LOW_RUNG(0);

            public final int ticks;

            LiftAction(int ticks) {
                this.ticks = ticks;
            }
        }

        enum OuttakeAction{
            INIT,
            HANG,
            TRANSFER,
            COLLECT,
            DISABLE,

            SCORE_HIGH_BASKET,
            SCORE_LOW_BASKET,
            SCORE_LOW_RUNG,
            SCORE_HIGH_RUNG,
        }
    }
}
