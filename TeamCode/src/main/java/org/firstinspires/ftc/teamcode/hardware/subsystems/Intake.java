package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

import java.util.function.DoubleSupplier;

public class Intake {

    public enum State{
        INTAKING,
        OUTTAKING,
        STOPPED;
    }

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double REVERSE_INTAKE_POWER = -1.0;
    public static double REVERSE_TRANSFER_POWER = -1.0;

    private final DcMotor intake, transfer;

    private State currentState = State.STOPPED;

    public Intake(@NonNull HardwareMap hwmap){
        intake = hwmap.get(DcMotor.class, HardwareConfig.intake);
        transfer = hwmap.get(DcMotor.class, HardwareConfig.transfer);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command activate(){
        return instant(() -> setPower(INTAKE_POWER, TRANSFER_POWER))
                .requiring(this);
    }

    public Command reverse(){
        return instant(() -> setPower(REVERSE_INTAKE_POWER, REVERSE_TRANSFER_POWER))
                .requiring(this);
    }

    public Command stop(){
        return instant(() -> setPower(0, 0))
                .requiring(this);
    }

    public Command toggle(){
        return instant(() -> {
            if(currentState == State.INTAKING){
                setPower(0.0, 0.0);
            } else {
                setPower(INTAKE_POWER, TRANSFER_POWER);
            }
        }).requiring(this);
    }

    public Command run(DoubleSupplier intakePow, DoubleSupplier transferPow){
        return instant(() -> setPower(intakePow.getAsDouble(), transferPow.getAsDouble()))
                .requiring(this);
    }

    private void setPower(double intakePow, double transferPow){
        intake.setPower(intakePow);
        transfer.setPower(transferPow);
        if(intakePow == 0 && transferPow == 0){
            currentState = State.STOPPED;
        } else if(intakePow > 0 && transferPow > 0){
            currentState = State.INTAKING;
        } else {
            currentState = State.OUTTAKING;
        }
    }

    public boolean isRunning(){
        return currentState != State.STOPPED;
    }

}
