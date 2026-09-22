package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.config.RobotConstants.IntakeState;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Intake implements Updateable {
    private final DcMotor intake, transfer;
    private IntakeState currentState = IntakeState.OFF;
    private IntakeState lastState = null;

    public Intake(@NonNull HardwareMap hwmap){
        intake = hwmap.get(DcMotor.class, HardwareConfig.intake);
        transfer = hwmap.get(DcMotor.class, HardwareConfig.transfer);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void toggle(){
        if(currentState == IntakeState.ON || currentState == IntakeState.SPIT){
            currentState = IntakeState.OFF;
        }
        else{
            currentState = IntakeState.ON;
        }
    }

    @Override
    public void update(){
        if(currentState != lastState){
            intake.setPower(currentState.val);
            transfer.setPower(currentState.val);

            lastState = currentState;
        }
    }

    public void setState(IntakeState state){
        currentState = state;
    }

    public IntakeState getState(){
        return currentState;
    }
}
