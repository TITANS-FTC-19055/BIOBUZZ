package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        controller1 = new Controller(gamepad1);
        robot = new Robot(hardwareMap);
        Scheduler.reset();
    }

    @Override
    public void loop() {
        robot.drivetrain.drive(-controller1.leftStickY, controller1.leftStickX, controller1.rightStickX);

        if(controller1.square.isPressed()) robot.toggleIntake();

        if(controller2.square.isPressed()) robot.toggleReversing();

        if(controller1.circle.isPressed()) robot.shoot();

        Scheduler.execute();

        robot.debug(telemetry);

        telemetry.update();
    }

    @Override
    public void stop(){
        Scheduler.reset();
    }
}

