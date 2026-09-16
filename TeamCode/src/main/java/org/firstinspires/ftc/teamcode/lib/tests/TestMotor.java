package org.firstinspires.ftc.teamcode.lib.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

@Disabled
@Config
@TeleOp(name = "TestMotor", group = "Test")
public class TestMotor extends LinearOpMode {

    float speed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");


        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            motor.setPower(speed);
            telemetry.addData("Motor position: ", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
