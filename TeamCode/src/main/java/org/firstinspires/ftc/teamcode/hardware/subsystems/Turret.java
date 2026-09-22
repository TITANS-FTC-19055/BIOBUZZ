package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Turret implements Updateable {

        public DcMotorEx launch1;
        public DcMotorEx launch2;
        private final CRServo hrot1, hrot2;
        public Servo block;


        public static double ks = 0, kv = 0.00055, ka = 100, kp = 0.008;
        public double target;
        //   public static double trage_shrek_far;
        public boolean runPid = false, toggle = false;

        // public Servo one,vertical;
        //  public Servo two;

        public shooting_states state;
        public DcMotorEx rotation_encoder;
        public double ticksPerRotation = 8192;
        public double rotation_target = 0;

        public double voltage, velocity, nominalVoltage = 9.3;
        PIDController controller = new PIDController(kp, 0, 0);
        PIDController rotationController = new PIDController(0,0,0);
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        public Turret(@NonNull HardwareMap hwmap) {
            rotation_encoder = hwmap.get(DcMotorEx.class, HardwareConfig.rotation_encoder);
            launch1 = hwmap.get(DcMotorEx.class, HardwareConfig.launch1);
            launch2 = hwmap.get(DcMotorEx.class, HardwareConfig.launch2);
            launch2.setDirection(DcMotorSimple.Direction.REVERSE);
            hrot1 = hwmap.get(CRServo.class, HardwareConfig.hrot1);
            hrot2 = hwmap.get(CRServo.class, HardwareConfig.hrot2);
            block = hwmap.get(Servo.class, HardwareConfig.block);
            launch1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            launch1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            block.setPosition(0);//servo-ul incepe blocand bilele
            state = shooting_states.OFF;
        }
        @Override
        public void update() {

            double rotation = rotation_encoder.getCurrentPosition();
            velocity = launch1.getVelocity();
            controller.setPID(kp, 0, 0);
            rotationController.setPID(0,0,0);
            feedforward = new SimpleMotorFeedforward(ks, kv, ka);
            double rotationPIDoutput = rotationController.calculate(rotation, rotation_target);
            double PID_output = controller.calculate(velocity, target);
            double ff_output = feedforward.calculate(target);
            launch1.setPower((PID_output + ff_output) * (nominalVoltage / voltage));
            launch2.setPower((PID_output + ff_output) * (nominalVoltage / voltage));
            hrot1.setPower(rotationPIDoutput);
            hrot2.setPower(rotationPIDoutput);
            if (launch1.getVelocity() < 100 && state == shooting_states.OFF) {
                launch1.setPower(0);
                launch2.setPower(0);
            }



        }

        public enum shooting_states {
            OFF(0),
            //this power is used to launch polen and nectar into the box
            SHOOTING_CLOSE(1200),
            //this power is used for launching polen and nectar into the goal
            SHOOTING_FAR(1440);

            public double val;

            shooting_states(double val) {
                this.val = val;
            }
        }

        public void setRotation(shooting_states statex) {

            if (state == statex) {
                state = shooting_states.OFF;
            } else state = statex;
            runPid = true;
            target = state.val;
        }
        public void setHorizontalAngle(double angleInDegrees){
            rotation_target = (8192*angleInDegrees)/360;
        }
    /*public void setAngle(double angle){
        one.setPosition(angle);two.setPosition(angle);
    }
    public void toggleAngle() {
        if (!toggle) setAngle(0.7);
        else setAngle(1);
        toggle = !toggle;
    }*/


    }