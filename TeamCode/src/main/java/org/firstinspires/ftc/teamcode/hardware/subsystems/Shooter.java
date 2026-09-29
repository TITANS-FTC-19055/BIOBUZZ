package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

import java.util.function.Supplier;

@Config
public class Shooter {

    private final DcMotorEx motor1, motor2;
    private final VoltageSensor voltageSensor;

    public static double ks = 0, kv = 0.00035, ka = 80, kp = 0.0001;
    public static double nominalVoltage = 12.0;

    private double targetVelocity = 0;
    private double currentVelocity = 0;
    private double voltage = 12.0;
    private volatile double boostPower = 0.0;

    // Tracks tuning parameters to avoid allocating objects inside high-frequency control loops
    private double lastKp = kp, lastKs = ks, lastKv = kv, lastKa = ka;

    private final PIDController controller;
    private SimpleMotorFeedforward feedforward;

    public Shooter(@NonNull HardwareMap hwmap) {
        motor1 = hwmap.get(DcMotorEx.class, HardwareConfig.launch1);
        motor2 = hwmap.get(DcMotorEx.class, HardwareConfig.launch2);

        voltageSensor = hwmap.voltageSensor.iterator().next();

        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDController(kp, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
    }

    private void refresh() {
        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            if (v > 1.0) {
                voltage = v;
            }
        }
        currentVelocity = motor1.getVelocity();

        checkTuningParameters();
    }

    private void checkTuningParameters() {
        if (kp != lastKp) {
            controller.setPID(kp, 0, 0);
            lastKp = kp;
        }
        if (ks != lastKs || kv != lastKv || ka != lastKa) {
            feedforward = new SimpleMotorFeedforward(ks, kv, ka);
            lastKs = ks;
            lastKv = kv;
            lastKa = ka;
        }
    }

    private double calculateMotorPower() {
        double pidOutput = controller.calculate(currentVelocity, targetVelocity);
        double ffOutput = feedforward.calculate(targetVelocity);

        double totalPower = (pidOutput + ffOutput + boostPower) * (nominalVoltage / voltage);
        return Range.clip(totalPower, -1.0, 1.0);
    }

    private void setMotorPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public Command update() {
        return infinite(() -> {
            refresh();
            setMotorPower(calculateMotorPower());
        });
    }

    public Command spin(Supplier<Double> targetVelocitySupplier) {
        return infinite(() -> {
            targetVelocity = targetVelocitySupplier.get();
            refresh();
            setMotorPower(calculateMotorPower());
        })
                .setEnd(endCondition -> setMotorPower(0))
                .requiring(this);
    }

    public Command waitUntilReady() {
        return Command.build().setDone(() -> isAtTargetVelocity(50.0));
    }

    public Command stop() {
        return instant(() -> {
            targetVelocity = 0;
            setMotorPower(0);
            controller.reset();
        }).requiring(this);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public boolean isAtTargetVelocity(double error) {
        return Math.abs(currentVelocity - targetVelocity) <= error;
    }
}