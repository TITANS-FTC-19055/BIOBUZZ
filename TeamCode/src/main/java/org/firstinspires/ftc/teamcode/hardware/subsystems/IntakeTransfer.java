package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

public class IntakeTransfer {
    private final DcMotor intake, transfer;
    private INTAKE_STATES state = INTAKE_STATES.OFF;

    public IntakeTransfer(@NonNull HardwareMap hwmap){
        intake = hwmap.get(DcMotor.class, HardwareConfig.intake);
        transfer = hwmap.get(DcMotor.class, HardwareConfig.transfer);
    }
    public enum INTAKE_STATES{
        ON(1),
        OFF(0),
        SPIT(-1);
        final double val;
         INTAKE_STATES(double val){
            this.val=val;
        }

    }

    public void update(){
        switch (state){
            case ON:
                intake.setPower(INTAKE_STATES.ON.val);
                transfer.setPower(INTAKE_STATES.ON.val);
                break;
            case OFF:
                intake.setPower(INTAKE_STATES.OFF.val);
                transfer.setPower(INTAKE_STATES.OFF.val);
                break;
            case SPIT:
                intake.setPower(INTAKE_STATES.SPIT.val);
                transfer.setPower(INTAKE_STATES.SPIT.val);
                break;
        }

    }
}
