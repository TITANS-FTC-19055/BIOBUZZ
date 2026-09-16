package org.firstinspires.ftc.teamcode.hardware.subsystems.templates;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

public class DrivetrainTemplate {

//    private final DcMotorEx RF, RB, LF, LB;
//
//    public DrivetrainTemplate(@NonNull HardwareMap hwmap){
//        RF = hwmap.get(DcMotorEx.class, HardwareConfig.RF);
//        LF = hwmap.get(DcMotorEx.class, HardwareConfig.LF);
//        RB = hwmap.get(DcMotorEx.class, HardwareConfig.RB);
//        LB = hwmap.get(DcMotorEx.class, HardwareConfig.LB);
//
//        RF.setDirection(DcMotorSimple.Direction.FORWARD);
//        LF.setDirection(DcMotorSimple.Direction.REVERSE);
//        RB.setDirection(DcMotorSimple.Direction.FORWARD);
//        LB.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void drive(double forward, double right, double rotate) {
//        RF.setPower(forward + right - rotate);
//        LF.setPower(forward - right + rotate);
//        RB.setPower(forward - right - rotate);
//        LB.setPower(forward + right + rotate);
//    }
//
//    public void setMotorPower(double rf_power,double lf_power, double rb_power, double lb_power){
//        RF.setPower(rf_power);
//        LF.setPower(lf_power);
//        RB.setPower(rb_power);
//        LB.setPower(lb_power);
//    }

}
