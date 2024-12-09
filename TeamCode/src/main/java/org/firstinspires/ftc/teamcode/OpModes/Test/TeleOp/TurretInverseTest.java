package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// love u Albert <3
@TeleOp(name = "TurretInverse", group = "test")
public class TurretInverseTest extends LinearOpMode {
    public static final double R = 0; //extensia maxima fata de centrul de rotatie (in cm)
    public static final double ROBOT_WIDTH = 0; //cm
    public static final double x = 50.8 - ROBOT_WIDTH / 2;

    public static final double turretMotorEncoderResolution = 145.1;
    public static final double turretGearRatio = 1.0 / 8;
    public static final double MAX_turretEncoderResolution =
            turretMotorEncoderResolution / turretGearRatio * 90 / 360; // regula de trei simpla

    public static final double extendoMotorEncoderResolution = 384.5;
    public static final double extendoGearRatio = 1.0 / 1;
    public static final double extendoRevolutionsToMaxExt = 0; //cm
    public static final double MAX_extendoEncoderResolution =
            extendoMotorEncoderResolution / extendoGearRatio * extendoRevolutionsToMaxExt; //sau doar da valoarea pe care o ai deja efectiv

    public static final double alpha0 = 0;
    private GamepadEx g1;


    //alpha (in grade, NU RADIANI)
    private double turretInverseKinematics(double alpha) {
        alpha = Math.min(90, Math.max(0, alpha));

        if (alpha <= alpha0)
            return R;

        // pentru ca imi trebuie unghiul fata de Ox, nu Oy
        // in contextul in care axa Oy coincide cu pozitia 0 a turretei
        double newAlpha = Math.toRadians(90 - alpha);

        return Math.tan(newAlpha) * x / Math.sin(newAlpha);
    }

    public void moveTurret(double alpha) {
        double turretPosition = alpha * MAX_turretEncoderResolution / 90;

        double legalExtendoPosition = turretInverseKinematics(alpha);
        double currentExtendoPosition = 0; //extendoMotor.getCurrentPosition() * R / MAX_extendoEncoderResolution;

        if (currentExtendoPosition > legalExtendoPosition)
            moveExtendo(legalExtendoPosition);

        /**inlocuieste cu your actual motor*/
        //turretMotor.setTargetPosition(position)
        //turretMotor.setPower(1)
    }

    // [0, R] cm
    public void moveExtendo(double distance) {
        distance = Math.max(R, Math.max(0, distance));
        //extendoMotor.setTargetPosition(distance * MAX_extendoEncoderResolution / R);
        //extendoMotor.setPower(1)
    }


    @Override
    public void runOpMode() throws InterruptedException {
        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            // [0,90] e range-ul
            moveTurret((g1.getLeftX() > 0) ? 90 * g1.getLeftX() : 0);

            // [0, R] e range-ul
            moveExtendo((g1.getRightY() > 0) ? R * g1.getRightY() : 0);

            g1.readButtons();
        }
    }
}
