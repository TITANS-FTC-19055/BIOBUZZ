package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

@TeleOp
public class Test extends LinearOpMode {
    private Robot robot;
    private Drivetrain drivetrain;
    private Controller controller1, controller2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            controller1.update();
            controller2.update();

            robot.drivetrain.driveFieldCentric(-controller1.leftStickX, controller1.leftStickY, controller1.rightStickX);
            robot.update();
            if(controller1.circle.isPressed()) robot.setState(Robot.ROBOT_STATES.COLLECTING);

            if(gamepad1.right_bumper) robot.setState(Robot.ROBOT_STATES.SHOOTING);
            telemetry.update();
        }
    }
}
