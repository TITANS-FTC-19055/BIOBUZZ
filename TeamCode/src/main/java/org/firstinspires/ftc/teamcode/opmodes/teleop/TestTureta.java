package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class TestTureta extends OpMode {
    Turret turret;
    Controller controller;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();

        if(controller.square.isPressed()) turret.goToAngle(90).schedule();
        if(controller.circle.isPressed()) turret.goToAngle(180).schedule();
        if(controller.triangle.isPressed()) turret.goToAngle(-90).schedule();

        if(controller.cross.isPressed()) turret.trackFieldPose(() -> new Pose(72, 72), () -> new Pose(72, 12)).schedule();
        if(controller.bumperLeft.isPressed()) turret.trackFieldPose(() -> new Pose(12, 72), () -> new Pose(72, 12)).schedule();
        if(controller.bumperRight.isPressed()) turret.trackFieldPose(() -> new Pose(132, 72), () -> new Pose(72, 12)).schedule();

        turret.debug(telemetry);

        Scheduler.execute();
        telemetry.update();
    }

    @Override
    public void stop(){
        Scheduler.reset();
    }
}