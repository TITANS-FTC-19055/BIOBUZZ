package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

import java.util.function.Supplier;

@Configurable
public class Turret {
    private final CRServo servo1, servo2;
    private final DcMotorEx encoder;

    public static double kPLeft = 0.02, kILeft = 0.00007, kDLeft = 0.00065;
    public static double kPRight = 0.025, kIRight = 0.0001, kDRight = 0.0008;

    public double GEAR_RATIO = 80.0/30.0;
    public double TICKS_PER_DEGREE = 8192.0/360.0 * GEAR_RATIO;

    public double MIN_ANGLE = -140;
    public double MAX_ANGLE = 140;

    private double targetAngle = 0;
    private double currentAngle = 0;

    private final PIDController controller;



    public Turret(@NonNull HardwareMap hwmap){
        servo1 = hwmap.get(CRServo.class, HardwareConfig.hrot1);
        servo2 = hwmap.get(CRServo.class, HardwareConfig.hrot2);

        encoder = hwmap.get(DcMotorEx.class, HardwareConfig.RF);
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(kPLeft, kILeft, kDLeft);
    }

    public void updateCurrentAngle(){
        currentAngle = encoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    private void setServoPower(double power){
        servo1.setPower(power);
        servo2.setPower(power);
    }

    private void rotateToAngle(double angle){
        updateCurrentAngle();

        targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);

        if(targetAngle - currentAngle > 0){
            controller.setPID(kPLeft, kILeft, kDLeft);
        } else {
            controller.setPID(kPRight, kIRight, kDRight);
        }

        if (isAtTargetAngle(1.0)) {
            setServoPower(0);
            return;
        }

        double pidPower = controller.calculate(currentAngle, targetAngle);

        setServoPower(Range.clip(pidPower, -1.0, 1.0));
    }

    public Command goToAngle(Supplier<Double> angleSupplier) {
        return infinite(() -> {
            rotateToAngle(angleSupplier.get());
        })
                .requiring(this);
    }

    public Command goToAngle(double targetAngle) {
        return goToAngle(() -> targetAngle);
    }

    public Command trackFieldPose(Supplier<Pose> target, Supplier<Pose> robot){
        return infinite(() -> {
            Pose t = target.get(), r = robot.get();

            double fieldTarget = Math.atan2(t.y() - r.y(), t.x() - r.x());
            double relativeTarget = fieldTarget - r.heading();

            rotateToAngle(-Math.toDegrees(relativeTarget));
        }).requiring(this);
    }

    public Command waitUntilReady(double toleranceDegrees) {
        return Command.build().setDone(() -> isAtTargetAngle(toleranceDegrees));
    }

    public Command waitUntilReady() {
        return waitUntilReady(2.0);
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

    public double getTargetAngle(){
        return targetAngle;
    }
    public double getAngleError() {
        return Math.abs(currentAngle - targetAngle);
    }

    public boolean isAtTargetAngle(double errorTolerance) {
        return getAngleError() <= errorTolerance;
    }

    public void debug(Telemetry t){
        updateCurrentAngle();
        t.addData("Current Angle", currentAngle);
        t.addData("Target Angle", targetAngle);
        t.addData("Angle Error", getAngleError());
        t.addData("Is At Target Angle", isAtTargetAngle(2.0));
        t.addData("kP", controller.getP());
        t.addData("kI", controller.getI());
        t.addData("kd", controller.getD());
        t.addData("power", servo1.getPower());
    }
}
