package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lib.control.Controller;

import java.util.function.DoubleSupplier;

@TeleOp
public class Prezentare extends OpMode {
    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    Shooter shooter;
    Controller controller;

    double currentTurretAngle = 0;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, false);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();

        drivetrain.drive(-controller.leftStickY, controller.leftStickX, controller.rightStickX);

        if(controller.square.isPressed()) intake.toggleIntaking().schedule();

        if(controller.circle.isPressed()) intake.toggleReversing().schedule();

        if(controller.cross.isPressed()) {
            parallel(
                    shooter.spin(() -> 1500.0),
                    sequential(
                            shooter.waitUntilReady(),
                            intake.feedToShooter()
                    )
            )
                    .until(() -> intake.isStorageEmpty())
                    .schedule();
        }


        if(controller.bumperRight.isPressed()){
            currentTurretAngle -= 30;
            turret.goToAngle(currentTurretAngle).schedule();
        }

        if(controller.bumperLeft.isPressed()){
            currentTurretAngle += 30;
            turret.goToAngle(currentTurretAngle).schedule();
        }


        Scheduler.execute();

        shooter.debug(telemetry);

        telemetry.update();
    }
    private double calculateTargetRpm(){


        return getRpmForDistance(150);
    }

    private double getRpmForDistance(double distance){

        return 1500;
    }


    @Override
    public void stop(){
        Scheduler.reset();
    }
}
