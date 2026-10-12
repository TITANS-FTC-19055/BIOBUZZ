package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

@TeleOp
public class Prezentare extends OpMode {
    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    Shooter shooter;
    Controller controller;

    boolean shooting = false;
    boolean intakeActive = false;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, false);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();

        drivetrain.drive(-controller.leftStickY, controller.leftStickX, controller.rightStickX);

        if(controller.square.isPressed()) {
            if(!intakeActive){
                intake.activate().schedule();
                intakeActive = true;
            } else {
                intake.stop().schedule();
                intakeActive = false;
            }
        }

        if(controller.triangle.isPressed()) {
            if(!intakeActive){
                intake.feedToShooter().schedule();
                intakeActive = true;

            } else {
                intake.stop().schedule();
                intakeActive = false;
            }
        }

        if(controller.circle.isPressed()){
            if(!intakeActive){
                intake.reverse().schedule();
                intakeActive = true;

            } else {
                intake.stop().schedule();
                intakeActive = false;
            }
        }

        if(controller.dpadUp.isPressed()) {
            if(!shooting){
                shooter.setPower(0.7);
                shooting = true;
            } else {
                shooter.setPower(0);
                shooting = false;
            }
        }

        if(controller.dpadRight.isPressed()) {
            if(!shooting){
                shooter.setPower(1);
                shooting = true;
            } else {
                shooter.setPower(0);
                shooting = false;
            }
        }

        if(controller.dpadLeft.isPressed()) {
            if(!shooting){
                shooter.setPower(0.5);
                shooting = true;
            } else {
                shooter.setPower(0);
                shooting = false;
            }
        }

        if(controller.dpadDown.isPressed()) {
            if(!shooting){
                shooter.setPower(0.3);
                shooting = true;
            } else {
                shooter.setPower(0);
                shooting = false;
            }
        }

        Scheduler.execute();
        telemetry.update();
    }

    @Override
    public void stop(){
        Scheduler.reset();
    }
}
