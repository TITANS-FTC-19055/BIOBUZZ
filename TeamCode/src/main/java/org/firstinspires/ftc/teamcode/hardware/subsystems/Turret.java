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

    public static double kP = 0.0155, kD = 0.00058;
    public static double kS = 0.06;          // smallest power that reliably moves the turret
    public static double kOmega = 0.0;       // power per deg/s of robot rotation
    public static double DEADBAND_DEG = 0.5; // kS switches off inside this error
    public static double MAX_POWER = 1.0;    // lower (e.g. 0.6) to soften big steps

    public double GEAR_RATIO = 80.0/30.0;
    public double TICKS_PER_DEGREE = 8192.0/360.0 * GEAR_RATIO;

    public double MIN_ANGLE = -140;
    public double MAX_ANGLE = 140;

    private double targetAngle = 0;
    private double currentAngle = 0;
    private double robotOmega = 0;

    private final PIDController controller;

    private double relativeTarget = 0;
    private double fieldTarget = 0;

    public Turret(@NonNull HardwareMap hwmap){
        servo1 = hwmap.get(CRServo.class, HardwareConfig.hrot1);
        servo2 = hwmap.get(CRServo.class, HardwareConfig.hrot2);

        encoder = hwmap.get(DcMotorEx.class, HardwareConfig.RF);
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(kP, 0, kD);
    }

    public void update(){
        update(0);
    }

    public void update(double degPerSec) {
        currentAngle = encoder.getCurrentPosition() / TICKS_PER_DEGREE;
        robotOmega = degPerSec;
        double error = targetAngle - currentAngle;

        controller.setPID(kP, 0, kD);
        double pid = controller.calculate(currentAngle, targetAngle);
        double ks  = Math.abs(error) > DEADBAND_DEG ? Math.signum(error) * kS : 0;
        double ff  = -kOmega * robotOmega;

        double power = Range.clip(pid, -MAX_POWER, MAX_POWER) + ks + ff;
        setServoPower(Range.clip(power, -1.0, 1.0));
    }

    private void setServoPower(double power){
        servo1.setPower(power);
        servo2.setPower(power);
    }

    private void rotateToAngle(double angle) {
        targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
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

            fieldTarget = Math.atan2(t.y() - r.y(), t.x() - r.x());
            relativeTarget = normalizeRadians(fieldTarget - r.heading());

            rotateToAngle(chooseReachableAngle(Math.toDegrees(relativeTarget)));
        }).requiring(this);
    }

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI)   angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double chooseReachableAngle(double angleDeg) {
        double best = angleDeg;
        double bestCost = Double.MAX_VALUE;

        for (int k = -1; k <= 1; k++) {
            double candidate = angleDeg + 360.0 * k;
            if (candidate < MIN_ANGLE || candidate > MAX_ANGLE) continue;

            double cost = Math.abs(candidate - currentAngle);
            if (cost < bestCost) {
                bestCost = cost;
                best = candidate;
            }
        }

        if (bestCost == Double.MAX_VALUE) {
            // Dead zone: no equivalent angle reachable, clamp to the nearest limit.
            double wrapped = angleDeg;
            double distToMin = Math.abs(wrapped - MIN_ANGLE);
            double distToMax = Math.abs(wrapped - MAX_ANGLE);
            // Compare using wrapped distance so we pick the physically closer limit
            distToMin = Math.min(distToMin, 360.0 - distToMin);
            distToMax = Math.min(distToMax, 360.0 - distToMax);
            best = distToMin < distToMax ? MIN_ANGLE : MAX_ANGLE;
        }

        return best;

    }

    public Command waitUntilReady(double toleranceDegrees) {
        return Command.build().setDone(() -> isAtTargetAngle(toleranceDegrees));
    }

    public Command waitUntilReady() {
        return waitUntilReady(5.0);
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
        t.addData("Current Angle", currentAngle);
        t.addData("Target Angle", targetAngle);
        t.addData("Angle Error", getAngleError());
        t.addData("Is At Target Angle", isAtTargetAngle(2.0));
        t.addData("kP", controller.getP());
        t.addData("kI", controller.getI());
        t.addData("kd", controller.getD());
        t.addData("power servo 1", servo1.getPower());
        t.addData("power servo 2", servo2.getPower());
        t.addData("Field Target", Math.toDegrees(fieldTarget));
        t.addData("Relative Target", Math.toDegrees(relativeTarget));
        t.addData("Robot Omega", robotOmega);
    }
}
