package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

import java.util.function.Supplier;

@Configurable
public class Shooter {

    private final DcMotorEx motor1, motor2;
    private final VoltageSensor voltageSensor;

    public static double ks = 0.017;
    public static double kv = 0.000445;
    public static double ka = 0;
    public static double kp = 0.0002;
    public static double nominalVoltage = 12.0;

    private double targetVelocity = 0;
    private double currentVelocity = 0;
    private double voltage = 12.0;
    private volatile double boostPower = 0.0;

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
    }

    private void setMotorPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    private void update() {
        refresh();
        setMotorPower(calculateMotorPower());
    }

    private double calculateMotorPower() {
        controller.setPID(kp, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);


        double pidOutput = controller.calculate(currentVelocity, targetVelocity);
        double ffOutput = feedforward.calculate(targetVelocity);

        double totalPower = (pidOutput + ffOutput + boostPower) * (nominalVoltage / voltage);
        return Range.clip(totalPower, -1.0, 1.0);
    }


    public Command spin(Supplier<Double> targetVelocitySupplier) {
        return infinite(() -> {
            targetVelocity = targetVelocitySupplier.get();
            update();
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

    public void debug(Telemetry telemetry){
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Voltage", voltage);
        telemetry.addData("Boost Power", boostPower);
        telemetry.addData("Motor Power", calculateMotorPower());
        telemetry.addData("KP", kp);
        telemetry.addData("KV", kv);
        telemetry.addData("KA", ka);
        telemetry.addData("KS", ks);
    }
}