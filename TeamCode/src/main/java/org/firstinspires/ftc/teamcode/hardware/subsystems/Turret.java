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

        private final DcMotorEx motor1, motor2;
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

        private final TurretHRot hrot;

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

            hrot = new TurretHRot(hwmap);

            state = ShootingState.OFF;
        }

        @Override
        public void update() {
            voltage = voltageSensor.getVoltage();
            currentVelocity = motor1.getVelocity();

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

            hrot.update();

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
            motor1.setPower(power);
            motor2.setPower(power);
        }

        private void stopMotors(){
            motor1.setPower(0);
            motor2.setPower(0);
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

        public void setHRotAngle(double angle){
            hrot.setTargetAngle(angle);
        }

}