package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class TurretHRot implements Updateable {
    private final CRServo servo1, servo2;
    private final DcMotorEx encoder;

    public static double kP = 0.03, kI = 0.0, kD = 0.001;
    public static double TICKS_PER_DEGREE = 8192.0/360.0;

    private double targetAngleDegrees = 0;
    private double currentAngleDegrees = 0;

    private final PIDController controller;

    public TurretHRot(@NonNull HardwareMap hwmap){
        servo1 = hwmap.get(CRServo.class, HardwareConfig.hrot1);
        servo2 = hwmap.get(CRServo.class, HardwareConfig.hrot2);

        servo2.setDirection(CRServo.Direction.REVERSE);

        encoder = hwmap.get(DcMotorEx.class, HardwareConfig.rotation_encoder);
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        controller = new PIDController(kP, kI, kD);
    }

    @Override
    public void update() {
        currentAngleDegrees = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        double rotationPower = controller.calculate(currentAngleDegrees, targetAngleDegrees);

        rotationPower = Math.max(-1.0, Math.min(1.0, rotationPower));

        if(isAtTargetAngle(1.0)){
            rotationPower = 0.0;
        }

        setServoPower(rotationPower);
    }

    private void setServoPower(double power){
        servo1.setPower(power);
        servo2.setPower(power);
    }

    public void setTargetAngle(double degrees){
        this.targetAngleDegrees = degrees;
    }

    public double getTargetAngle(){
        return targetAngleDegrees;
    }

    public double getCurrentAngle(){
        return currentAngleDegrees;
    }

    public boolean isAtTargetAngle(double error){
        return Math.abs(currentAngleDegrees - targetAngleDegrees) <= error;
    }

    public void stop(){
        setServoPower(0.0);
        controller.reset();
    }
}
