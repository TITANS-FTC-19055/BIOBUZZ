package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

public class Drivetrain {
    private final DcMotorEx RF, RB, LF, LB;
    private IMU imu;

    private ElapsedTime timer = new ElapsedTime();

    public Drivetrain(@NonNull HardwareMap hwmap, boolean fieldCentric) {
        RF = hwmap.get(DcMotorEx.class, HardwareConfig.RF);
        LF = hwmap.get(DcMotorEx.class, HardwareConfig.LF);
        RB = hwmap.get(DcMotorEx.class, HardwareConfig.RB);
        LB = hwmap.get(DcMotorEx.class, HardwareConfig.LB);

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : new DcMotorEx[]{RF, LF, RB, LB}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(fieldCentric){
            imu = hwmap.get(IMU.class, "imu");

            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            );

            imu.initialize(new IMU.Parameters(orientation));
            imu.resetYaw();
        }
    }

    public void driveFieldCentric(double forward, double strafe, double rotate){
        if(imu == null){
            drive(forward, strafe, rotate);
            return;
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double rotatedForward = forward * cos - strafe * sin;
        double rotatedStrafe = forward * sin + strafe * cos;

        drive(rotatedForward, rotatedStrafe, rotate);
    }

    public void drive(double forward, double strafe, double rotate) {
        double rfPower = forward - strafe - rotate;
        double lfPower = forward + strafe + rotate;
        double rbPower = forward + strafe - rotate;
        double lbPower = forward - strafe + rotate;

        normalizeAndSet(rfPower, lfPower, rbPower, lbPower);
    }

    public void normalizeAndSet(double rf, double lf, double rb, double lb) {
        double max = Math.max(1.0, Math.max(Math.abs(rf), Math.max(Math.abs(lf), Math.max(Math.abs(rb), Math.abs(lb)))));
        setMotorPower(rf / max, lf / max, rb / max, lb / max);
    }

    public void setMotorPower(double rf_power,double lf_power, double rb_power, double lb_power) {
        RF.setPower(rf_power);
        LF.setPower(lf_power);
        RB.setPower(rb_power);
        LB.setPower(lb_power);
    }

    public void driveForward(double power) {
        setMotorPower(power, power, power, power);
    }

    public void driveBackward(double power) {
        setMotorPower(-power, -power, -power, -power);
    }

    public void strafeRight(double power) {
        setMotorPower(-power, power, power, -power);
    }

    public void strafeLeft(double power) {
        setMotorPower(power, -power, -power, power);
    }

    public void turnRight(double power) {
        setMotorPower(-power, power, -power, power);
    }

    public void turnLeft(double power) {
        setMotorPower(power, -power, power, -power);
    }

    public void forwardStrafeRight(double power) {
        setMotorPower(0, power, power, 0);
    }

    public void forwardStrafeLeft(double power) {
        setMotorPower(power, 0, 0, power);
    }

    public void backwardStrafeRight(double power) {
        setMotorPower(-power, 0, 0, -power);
    }

    public void backwardStrafeLeft(double power) {
        setMotorPower(0, -power, -power, 0);
    }

    public void stop() {
        setMotorPower(0, 0, 0, 0);
    }

}
