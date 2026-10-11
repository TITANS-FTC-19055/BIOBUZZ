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

    Pose pose;
    Pose robotPose = new Pose(72, 12, Math.toRadians(90));

    // Moving target state
    public static double SWEEP_TIME_S = 6;   // seconds to go from x=0 to x=144
    public static double START_X = 0, END_X = 144, TARGET_Y = 72;
    boolean moving = false;
    double sweepStartTime = 0;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new Controller(gamepad1);
        pose = new Pose(0, TARGET_Y);
    }

    @Override
    public void loop() {
        controller.update();

        if(controller.square.isPressed()) turret.goToAngle(90).schedule();
        if(controller.circle.isPressed()) turret.goToAngle(180).schedule();
        if(controller.triangle.isPressed()) turret.goToAngle(-90).schedule();

        if(controller.cross.isPressed()) turret.trackFieldPose(() -> new Pose(72, 72), () -> robotPose).schedule();
        if(controller.bumperLeft.isPressed()) turret.trackFieldPose(() -> new Pose(12, 72), () -> robotPose).schedule();
        if(controller.bumperRight.isPressed()) turret.trackFieldPose(() -> new Pose(132, 72), () -> robotPose).schedule();

        if(controller.dpadUp.isPressed()) {
            moving = true;
            sweepStartTime = getRuntime();
            turret.trackFieldPose(() -> pose, () -> robotPose).schedule();
        }
        // dpadDown: stop the sweep
        if(controller.dpadDown.isPressed()) moving = false;

        if(moving) {
            double t = (getRuntime() - sweepStartTime) / SWEEP_TIME_S;   // 0..1 linear progress
            double x = START_X + (END_X - START_X) * Math.min(t, 1.0);
            pose = new Pose(x, TARGET_Y);
            if(t >= 1.0) moving = false;                                  // reached 144, stop
        }

        telemetry.addData("X", pose.x());
        turret.debug(telemetry);

        Scheduler.execute();
        telemetry.update();
    }

    @Override
    public void stop(){
        Scheduler.reset();
    }
}