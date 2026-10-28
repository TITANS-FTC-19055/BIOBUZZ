package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class TestTureta extends OpMode {
    Turret turret;
    Controller controller;

    public static double targetAngle = 0;

    Pose pose;
    Pose robotPose = new Pose(72, 12, Math.toRadians(90));

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();

        if(controller.triangle.isPressed()) turret.goToAngle(targetAngle).schedule();
        if(controller.cross.isPressed()) turret.goToAngle(0).schedule();

        turret.update();

        turret.debug(telemetry);

        Scheduler.execute();

        telemetry.update();
    }

    @Override
    public void stop(){
        Scheduler.reset();
    }
}