package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

@TeleOp
public class TeleOpTemplate extends LinearOpMode {
    private Robot robot;
    private Controller controller1, controller2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            // Robot update trebuie sa ramana primul pentru a citi datele de la sensori
            robot.update();

            controller1.update();
            controller2.update();

            // Restul codului aici

            telemetry.update();
        }
    }
}
