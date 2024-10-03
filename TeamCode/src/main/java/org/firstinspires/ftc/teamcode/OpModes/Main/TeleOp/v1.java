package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;

@TeleOp(group = "main", name = "🍓")
public class v1 extends ExoMode {
    private Machine robot;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        WaitForStart();
        WhenStarted();

        Loop();
    }

    @Override
    protected void Init() {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.Telemetry.DASHBOARD)
                        .add(Enums.OpMode.TELE_OP)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false))
                .addGamepads(Enums.Gamepads.BOTH)
                .construct(this);
    }

    @Override
    protected void WhenStarted() { robot.clearTelemetry(); }

    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {
        while (opModeIsActive()) {
            robot.read();
            robot.update();
            robot.write();
        }
    }
}
