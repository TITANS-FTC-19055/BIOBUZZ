package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

@TeleOp
public class TestShooter extends OpMode {
    Controller controller;
    DcMotor motor1, motor2;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, HardwareConfig.launch1);
        motor2 = hardwareMap.get(DcMotor.class, HardwareConfig.launch2);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();

        if(controller.dpadUp.isPressed()) motor1.setPower(1);
        if(controller.dpadDown.isPressed()) motor1.setPower(-1);

        if(controller.dpadRight.isPressed()) motor2.setPower(1);
        if(controller.dpadLeft.isPressed()) motor2.setPower(-1);

        if(controller.cross.isPressed()) {
            motor1.setPower(0);
            motor2.setPower(0);
        }

    }
}
