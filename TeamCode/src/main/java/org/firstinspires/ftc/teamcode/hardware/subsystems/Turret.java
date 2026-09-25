package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;

public class Turret {

        private final DcMotorEx motor1, motor2;
        private final VoltageSensor voltageSensor;

        public static double ks = 0, kv = 0.00055, ka = 100, kp = 0.008;
        private double targetVelocity = 0, currentVelocity = 0;
        public double voltage = 12.0;
        public final double nominalVoltage = 12.0;
        private volatile double boostPower = 0.0;

        private final PIDController controller;
        private final SimpleMotorFeedforward feedforward;

        public Turret(@NonNull HardwareMap hwmap) {
            motor1 = hwmap.get(DcMotorEx.class, HardwareConfig.launch1);
            motor2 = hwmap.get(DcMotorEx.class, HardwareConfig.launch2);

            this.voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);

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

        private void refresh(){
            voltage = voltageSensor.getVoltage();
            currentVelocity = motor1.getVelocity();
        }

        private double calculateMotorPower(){
            double pidOutput = controller.calculate(currentVelocity, targetVelocity);
            double ffOutput = feedforward.calculate(targetVelocity);

            return Math.min(1.0, (pidOutput + ffOutput + boostPower) * (nominalVoltage / voltage));
        }

        private void setMotorPower(double power){
            motor1.setPower(power);
            motor2.setPower(power);
        }

        public Command spinTo(double targetVelocity){
            return infinite(() -> {
                refresh();
                setMotorPower(calculateMotorPower());
            })
                    .setStart(() -> this.targetVelocity = targetVelocity)
                    .requiring(this);
        }

        public Command waitUntilReady(){
            return Command.build().setDone(() -> isAtTargetVelocity(50.0));
        }

        public Command boost(double boost){
            return instant(() -> boostPower = boost);
        }

        public Command stop(){
            return instant(() -> {
                motor1.setPower(0);
                motor2.setPower(0);
                controller.reset();
            }).requiring(this);
        }

        public boolean isAtTargetVelocity(double error){
            return Math.abs(currentVelocity - targetVelocity) <= error;
        }

}