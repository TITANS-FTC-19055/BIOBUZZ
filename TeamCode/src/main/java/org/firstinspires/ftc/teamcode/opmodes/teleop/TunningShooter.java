package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.lib.control.Controller;


@TeleOp
public class TunningShooter extends OpMode {
    Shooter shooter;
    Controller controller;

    public static double target = 2000;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        controller = new Controller(gamepad1);
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Scheduler.reset();
    }

    @Override
    public void loop() {
        controller.update();

        if(controller.dpadRight.isPressed()) target = 2000;
        if(controller.dpadUp.isPressed()) target = 1500;
        if(controller.dpadLeft.isPressed()) target = 1000;
        if(controller.dpadDown.isPressed()) target = 500;


        if(controller.cross.isPressed()) shooter.stop().schedule();
        if(controller.square.isPressed()) shooter.spin(() -> target).schedule();

        Scheduler.execute();
        shooter.debug(telemetry);
        telemetry.update();
    }

    @Override
    public void stop(){
        Scheduler.reset();
    }

}
