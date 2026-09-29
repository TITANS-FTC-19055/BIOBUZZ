package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

@TeleOp
public class TestImaginar extends OpMode {
    Shooter shooter;
    Controller controller;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        controller = new Controller(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start(){
        shooter.spin(() -> 800.0).schedule();
    }

    @Override
    public void loop() {
        controller.update();

        telemetry.addData("Is at target:", shooter.isAtTargetVelocity(50));
        telemetry.addData("Target:", shooter.getTargetVelocity());
        telemetry.addData("Current:", shooter.getCurrentVelocity());

        Scheduler.execute();
        telemetry.update();
    }
}
