package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.control.Controller;


@TeleOp
public class Test extends OpMode {
    Controller controller1;
    Controller controller2;
    Robot robot;

    @Override
    public void init() {
        Scheduler.reset();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        robot = new Robot(hardwareMap);
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();

        robot.driveFieldCentric(controller1.leftStickX, -controller1.leftStickY, -controller1.rightStickX);

        if(controller1.square.isPressed()) robot.toggleIntake();

        if(controller1.cross.isPressed()) robot.shoot();

        if(controller1.triangle.isPressed()) robot.shootWithTracking(new Pose(72, 72));

        if(controller1.circle.isPressed()) robot.toggleContinousShootWithTracking(new Pose(72, 72));


        if(controller2.square.isPressed()) robot.toggleReversing();


        Scheduler.execute();

        robot.update();

        robot.debug(telemetry);
        telemetry.update();
    }

    @Override
    public void stop(){
        Scheduler.reset();
    }
}

