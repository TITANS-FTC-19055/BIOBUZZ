package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class TestTureta extends OpMode {
    CRServo servo1, servo2;
    Controller controller;
    @Override
    public void init() {
        servo1 = hardwareMap.get(CRServo.class, HardwareConfig.hrot1);
        servo2 = hardwareMap.get(CRServo.class, HardwareConfig.hrot2);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();
        if(controller.square.isDown()){
            servo1.setPower(1.0);
        } else if(controller.circle.isDown()) {
            servo2.setPower(1.0);
        } else if(controller.cross.isDown()){
            servo1.setPower(1.0);
            servo2.setPower(1.0);
        } else {
            servo1.setPower(0.0);
            servo2.setPower(0.0);
        }

    }
}
