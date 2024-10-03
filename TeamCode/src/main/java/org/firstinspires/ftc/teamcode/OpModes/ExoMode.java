package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ExoMode extends LinearOpMode {
    protected abstract void Init();

    protected abstract void WhenStarted();

    protected abstract void InitializeThreads();

    protected abstract void Loop();

    protected void WaitForStart() { super.waitForStart(); }

    protected void AutonomusTasks() { idle(); }
}