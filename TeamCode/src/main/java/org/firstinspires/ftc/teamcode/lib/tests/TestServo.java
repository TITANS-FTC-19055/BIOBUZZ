package org.firstinspires.ftc.teamcode.lib.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "TestServo", group = "Test")
public class TestServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
         Servo servo = hardwareMap.get(Servo.class, "servo");
         servo.setDirection(Servo.Direction.FORWARD);

         waitForStart();

         servo.setPosition(1);

    }
}
