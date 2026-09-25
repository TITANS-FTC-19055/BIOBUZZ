package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

public class TurretHRot {
    private final CRServo servo1, servo2;
    private final DcMotorEx encoder;

    public static double kP = 0.03, kI = 0.0, kD = 0.001;
    //Placeholder
    public static  double GEAR_RATIO = 100.0/20.0;
    public static double TICKS_PER_DEGREE = 8192.0/360.0 * GEAR_RATIO;

    // Placeholder
    public static double MIN_ANGLE = -180;
    public static double MAX_ANGLE = 180;

    private double targetAngle = 0;
    private double currentAngle = 0;

    private final PIDController controller;

    public TurretHRot(@NonNull HardwareMap hwmap){
        servo1 = hwmap.get(CRServo.class, HardwareConfig.hrot1);
        servo2 = hwmap.get(CRServo.class, HardwareConfig.hrot2);

        servo2.setDirection(CRServo.Direction.REVERSE);

        encoder = hwmap.get(DcMotorEx.class, HardwareConfig.rotation_encoder);
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        controller = new PIDController(kP, kI, kD);
    }

    private void rotateToAngle(double angle){
        currentAngle = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        double normalized = AngleUnit.normalizeDegrees(angle);
        targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, normalized));

        double power = Math.max(-1.0, Math.min(1.0, controller.calculate(currentAngle, targetAngle)));
        setServoPower(isAtTargetAngle(1.0) ? 0.0 : power);
    }

    public Command trackFieldPose(Supplier<Pose> target, Supplier<Pose> robot){
        return infinite(() -> {
            Pose t = target.get(), r = robot.get();
            double fieldTarget = Math.atan2(t.y() - r.y(), t.x() - r.x());
            rotateToAngle(Math.toDegrees(fieldTarget - r.heading()));
        }).requiring(this);
    }

    public Command stop(){
        return instant(() -> {
            setServoPower(0);
            controller.reset();
        }).requiring(this);
    }

    public double getCurrentAngle(){
        return currentAngle;
    }

    public boolean isAtTargetAngle(double error){
        return Math.abs(currentAngle - targetAngle) <= error;
    }

    private void setServoPower(double power){
        servo1.setPower(power);
        servo2.setPower(power);
    }
}
