package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Constants.RobotState;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.lib.control.Controller;


@TeleOp
public class Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        Controller controller1 = new Controller(gamepad1);
        Controller controller2 = new Controller(gamepad2);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            controller1.update();
            controller2.update();

            robot.drive(-controller1.leftStickX, controller1.leftStickY, controller1.rightStickX);
            if(controller1.circle.isPressed()) robot.setState(RobotState.COLLECTING);
            if(controller1.square.isPressed()) robot.setState(RobotState.SHOOTING);

            robot.update();
            telemetry.update();
        }
    }
}

