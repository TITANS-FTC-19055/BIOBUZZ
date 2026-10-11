package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.control.Controller;


@TeleOp
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        Controller controller = new Controller(gamepad1);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            controller.update();

            robot.drive(-controller.leftStickX, controller.leftStickY, controller.rightStickX);

            // if(controller.circle.isPressed()) robot.intake.toggle().schedule();
            if(controller.square.isPressed()) robot.shootSequence().schedule();


            robot.update();
            robot.debug(telemetry);

            Scheduler.execute();

            telemetry.update();
        }
    }
}

