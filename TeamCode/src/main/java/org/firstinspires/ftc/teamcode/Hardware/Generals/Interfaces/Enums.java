package org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;

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

    enum Access{
        INTAKE,
        OUTTAKE
    }

    enum Telemetry{
        DASHBOARD,
        REGULAR
    }

    enum Gamepads{
        G1, G2,
        BOTH,
        NONE
    }


    enum Color{
        RED,
        BLUE,
        YELLOW,
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
            THREE_WHEELS,
            TWO_WHEELS
        }
    }

    interface IntakeEnums {
        enum IntakeAction{
            COLLECT,
            PRE_COLLECT,
            TRANSFER,
            DISABLE
        }

        enum IntakePosition{
            ZERO(0),
            IN_TRANSFER(200);

            public final int ticks;

            IntakePosition(int ticks) { this.ticks = ticks; }
        }
    }

    interface OuttakeEnums {
        enum ArmAction{
            DISABLE,
            PRE_TRANSFER,
            TRANSFER,
            PRE_SCORE,
            SCORE_SPECIMENS,
            SCORE_SAMPLES
        }

        enum LiftAction{
            ZERO(0),
            HANG(100),
            TRANSFER(0),
            FULL(outtakeMAX),

            HIGH_BASKET(1540),
            HIGH_RUNG(350),
            LOW_BASKET(0), // we don't do that here
            LOW_RUNG(0); // neither this

            public final int ticks;

            LiftAction(int ticks) {
                this.ticks = ticks;
            }
        }

        enum OuttakeAction{
            INIT,
            HANG,
            SCORE,
            PRE_TRANSFER,
            TRANSFER,

            PRE_SCORE,
            SCORE_HIGH_BASKET,
            SCORE_LOW_BASKET,
            SCORE_LOW_RUNG,
            SCORE_HIGH_RUNG,
        }
    }
}
