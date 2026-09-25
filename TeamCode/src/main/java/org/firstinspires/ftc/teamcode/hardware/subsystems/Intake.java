package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

public class Intake {
    private final DcMotor intake, transfer;


    public Intake(@NonNull HardwareMap hwmap){
        intake = hwmap.get(DcMotor.class, HardwareConfig.intake);
        transfer = hwmap.get(DcMotor.class, HardwareConfig.transfer);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command collect(){
        return instant(() -> {
            intake.setPower(1);
            transfer.setPower(1);
        }).requiring(this);
    }

    public Command stop(){
        return instant(() -> {
            intake.setPower(0);
            transfer.setPower(0);
        }).requiring(this);
    }

}
