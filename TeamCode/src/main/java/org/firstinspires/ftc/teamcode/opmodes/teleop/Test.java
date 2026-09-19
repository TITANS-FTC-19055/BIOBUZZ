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

        drivetrain = new Drivetrain(hardwareMap, true);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            controller1.update();
            controller2.update();

            drivetrain.driveFieldCentric(-controller1.leftStickY, controller1.leftStickX, controller1.rightStickX);

            if(controller1.dpadUp.isDown()) drivetrain.forwardStrafeRight(0.5);
            if(controller1.dpadDown.isDown()) drivetrain.forwardStrafeLeft(0.5);
            if(controller1.dpadLeft.isDown()) drivetrain.backwardStrafeRight(0.5);
            if(controller1.dpadRight.isDown()) drivetrain.backwardStrafeLeft(0.5);

            if(controller1.square.isDown()) drivetrain.turnLeft(0.5);
            if(controller1.circle.isDown()) drivetrain.turnRight(0.5);

            robot.update();
            telemetry.update();
        }
    }
}
