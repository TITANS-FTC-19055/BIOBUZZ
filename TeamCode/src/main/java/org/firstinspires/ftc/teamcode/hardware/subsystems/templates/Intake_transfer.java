package org.firstinspires.ftc.teamcode.hardware.subsystems.templates;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

public class Intake_transfer {
    public DcMotor intake, transfer;
    public Intake_states state=Intake_states.OFF;

    public Intake_transfer (HardwareMap HWMAP){
        intake=HWMAP.get(DcMotor.class,HardwareConfig.Intake);
        transfer=HWMAP.get(DcMotor.class,HardwareConfig.Transfer);
    }
    public enum Intake_states{
        ON(1),
        OFF(0),
        SPIT(-1);
        public double val;
         Intake_states(double val){
            this.val=val;
        }

    }
    public void update(){
        if (state==Intake_states.OFF){
            intake.setPower(0);
            transfer.setPower(0);
        }
        if (state==Intake_states.ON){
            intake.setPower(1);
            transfer.setPower(1);
        }
        if( state==Intake_states.SPIT){
            intake.setPower(-1);
            transfer.setPower(-1);
        }

    }
}
