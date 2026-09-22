package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.config.RobotConstants.ShootingState;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Turret implements Updateable {

        private final DcMotorEx launch1, launch2, rotation_encoder;
        private final CRServo hrot1, hrot2;
        private final VoltageSensor voltageSensor;

        public static double ks = 0, kv = 0.00055, ka = 100, kp = 0.008;

        private double targetVelocity = 0;
        private double currentVelocity = 0;
        public double voltage = 12.0;
        public final double nominalVoltage = 12.0;

        public double ticksPerRotation = 8192;
        public double rotation_target = 0;

        private ShootingState state;

        private final PIDController controller;
        private final SimpleMotorFeedforward feedforward;

        public Turret(@NonNull HardwareMap hwmap) {
            launch1 = hwmap.get(DcMotorEx.class, HardwareConfig.launch1);
            launch2 = hwmap.get(DcMotorEx.class, HardwareConfig.launch2);

            hrot1 = hwmap.get(CRServo.class, HardwareConfig.hrot1);
            hrot2 = hwmap.get(CRServo.class, HardwareConfig.hrot2);
            rotation_encoder = hwmap.get(DcMotorEx.class, HardwareConfig.rotation_encoder);

            this.voltageSensor = hwmap.getAll(VoltageSensor.class).get(0);

            launch1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            launch1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            launch2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            launch2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            launch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            launch2.setDirection(DcMotorSimple.Direction.REVERSE);

            controller = new PIDController(kp, 0, 0);
            feedforward = new SimpleMotorFeedforward(ks, kv, ka);

            state = ShootingState.OFF;
        }

        @Override
        public void update() {
            voltage = voltageSensor.getVoltage();
            currentVelocity = launch1.getVelocity();

//            controller.setPID(kp, 0, 0);

            switch (state){
                case OFF:
                    stopMotors();
                    break;
                case SPINNING_UP:
                    setMotorPower(calculateMotorPower());
                    if(isAtTargetVelocity(50.0)) {
                        state = ShootingState.READY;
                    }
                    break;
                case READY:
                    setMotorPower(calculateMotorPower());
                    break;
                case SHOOTING:
                    setMotorPower(calculateMotorPower());
                    if(!isAtTargetVelocity(120.0)){
                        state = ShootingState.SPINNING_UP;
                    }
                    break;
            }

        }

        private double calculateMotorPower(){
            double pidOutput = controller.calculate(currentVelocity, targetVelocity);
            double ffOutput = feedforward.calculate(targetVelocity);

//            if(state == ShootingState.SHOOTING && (targetVelocity - currentVelocity) > 100){
//                pidOutput += 0.15;
//            }

            return Math.min(1.0, (pidOutput + ffOutput) * (nominalVoltage / voltage));
        }

        private void setMotorPower(double power){
            launch1.setPower(power);
            launch2.setPower(power);
        }

        private void stopMotors(){
            launch1.setPower(0);
            launch2.setPower(0);
            controller.reset();
        }

        public void setState(ShootingState state){
            this.state = state;
        }

        public void setTargetVelocity(double targetVelocity){
            this.targetVelocity = targetVelocity;
        }

        public double getCurrentVelocity(){
            return currentVelocity;
        }

        public boolean isAtTargetVelocity(double error){
            return Math.abs(currentVelocity - targetVelocity) <= error;
        }

        public void setHorizontalAngle(double angleInDegrees){
            rotation_target = (8192*angleInDegrees)/360;
        }

    }