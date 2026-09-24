package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.config.Constants.*;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class TurretHRot implements Updateable {
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

    @Override
    public void update() {
        currentAngle = encoder.getCurrentPosition() / TICKS_PER_DEGREE;

        double rotationPower = controller.calculate(currentAngle, targetAngle);

        rotationPower = Math.max(-1.0, Math.min(1.0, rotationPower));

        if(isAtTargetAngle(1.0)){
            rotationPower = 0.0;
        }

        setServoPower(rotationPower);
    }

    public void setTargetAngleRobotCentric(double angle){
        double normalized = AngleUnit.normalizeDegrees(angle);
        this.targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, normalized));
    }

    public void setTargetAngleFieldCentric(Pose target, Pose robot){
        double dx = target.x() - robot.x();
        double dy = target.y() - robot.y();
        double fieldTarget = Math.atan2(dy, dx);
        double robotRelativeAngle = fieldTarget - robot.heading();
        setTargetAngleRobotCentric(Math.toDegrees(robotRelativeAngle));
    }

    public double getTargetAngle(){
        return targetAngle;
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

    public void stop(){
        setServoPower(0.0);
        controller.reset();
    }
}
