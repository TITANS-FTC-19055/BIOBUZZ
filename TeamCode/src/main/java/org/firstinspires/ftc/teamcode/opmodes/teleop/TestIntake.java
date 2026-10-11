package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

@TeleOp
public class TestIntake extends OpMode {
    public Intake intake;
    public Controller controller;

    public DcMotor motor;

    public void init(){
        intake = new Intake(hardwareMap);
        motor = hardwareMap.get(DcMotor.class, HardwareConfig.launch1);
        controller = new Controller(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void loop(){
        controller.update();

        if(controller.cross.isPressed()) intake.stop().schedule();
        if(controller.square.isPressed()) intake.activate().schedule();
        if(controller.circle.isPressed()) intake.reverse().schedule();
        if(controller.triangle.isPressed()) intake.feedToShooter().schedule();

        if(controller.dpadUp.isPressed()) motor.setPower(-0.5);
        if(controller.dpadDown.isPressed()) motor.setPower(-0.7);
        if(controller.dpadRight.isPressed()) motor.setPower(-1);
        if(controller.dpadLeft.isPressed()) motor.setPower(0);

        intake.debug(telemetry);

        Scheduler.execute();
        telemetry.update();
    }

}
