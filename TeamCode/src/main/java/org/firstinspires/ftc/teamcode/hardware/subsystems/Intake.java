package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

@Config
public class Intake {

    public enum State {
        INTAKING,
        OUTTAKING,
        SHOOTING,
        STOPPED
    }

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double REVERSE_INTAKE_POWER = -1.0;
    public static double REVERSE_TRANSFER_POWER = -1.0;

    private boolean isFull = false;
    private boolean isEmpty = true;

    public static double INTAKE_SENSOR_THRESHOLD_CM = 10.0;
    public static double EXIT_SENSOR_THRESHOLD_CM = 5.0;

    private static final long REVERSE_RESET_DURATION_NS = 200_000_000L;
    private static final long INTAKE_CONFIRMATION_DURATION_NS = 800_000_000L;
    private static final long FEED_CONFIRMATION_DURATION_NS = 400_000_000L;
    private static final long BLOCKER_FEED_DURATION_NS = 1_000_000_000L;

    private static long intakeStartTime = 0;
    private boolean startedConfirmingFull = false;
    private long feedStartTime = 0;
    private long reverseStartTime = 0;
    private boolean startedConfirmingEmpty = false;
    private long blockerStartTime = 0;
    private boolean blocked = false;

    private final DcMotor intake, transfer;
    private final DistanceSensor intakeSensor, exitSensor;
    private final Servo blocker;

    private State currentState = State.STOPPED;
    private boolean intakeTriggered = false;
    private boolean exitTriggered = false;

    public Intake(@NonNull HardwareMap hwmap) {
        intake = hwmap.get(DcMotor.class, HardwareConfig.intake);
        transfer = hwmap.get(DcMotor.class, HardwareConfig.transfer);

        intakeSensor = hwmap.get(DistanceSensor.class, HardwareConfig.intake_sensor);
        exitSensor = hwmap.get(DistanceSensor.class, HardwareConfig.exit_sensor);

        blocker = hwmap.get(Servo.class, HardwareConfig.blocker);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        blocker.setPosition(1);
    }

    public Command activate() {
        return infinite(() -> {
            run(State.INTAKING, INTAKE_POWER, TRANSFER_POWER);
        })
                //.until(this::isStorageFull)
                //.unless(this::isStorageFull)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command reverse() {
        return infinite(() -> {
            run(State.OUTTAKING, REVERSE_INTAKE_POWER, REVERSE_TRANSFER_POWER);
        })
                //.unless(this::isStorageEmpty)
                //.until(this::isStorageEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command feedToShooter() {
        return infinite(() -> {
            run(State.SHOOTING, INTAKE_POWER, TRANSFER_POWER);
        })
                //.until(this::isStorageEmpty)
                //.unless(this::isStorageEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command stop() {
        return instant(this::halt).requiring(this);
    }

    private void run(State state, double intakePow, double transferPow) {
        if(currentState != state && state == State.OUTTAKING){
            reverseStartTime = System.nanoTime();
            startedConfirmingEmpty = false;
        }

        if(currentState != state && state == State.INTAKING){
            blocker.setPosition(1);
            blocked = true;
            startedConfirmingFull = false;
        }

        if(currentState != state && state == State.SHOOTING){
            blockerStartTime = System.nanoTime();
            startedConfirmingEmpty = false;
        }

        currentState = state;

        if((System.nanoTime() - blockerStartTime >= BLOCKER_FEED_DURATION_NS) && blocked && state == State.SHOOTING){
          blocker.setPosition(0.7762);
          blocked = false;
        }

        update();

        intake.setPower(intakePow);
        transfer.setPower(transferPow);
    }

    private void halt() {
        intake.setPower(0);
        transfer.setPower(0);
        currentState = State.STOPPED;
    }

    public void update() {
        long now = System.nanoTime();

        boolean newIntake = intakeSensor.getDistance(DistanceUnit.CM) < INTAKE_SENSOR_THRESHOLD_CM;
        boolean newExit   = exitSensor.getDistance(DistanceUnit.CM)   < EXIT_SENSOR_THRESHOLD_CM;

        switch (currentState) {
            case INTAKING:
                if(newIntake && intakeTriggered && !startedConfirmingFull){
                    intakeStartTime = System.nanoTime();
                    startedConfirmingFull = true;
                } else if(!newIntake && startedConfirmingFull){
                    intakeStartTime = 0;
                    startedConfirmingFull = false;
                }

                if(startedConfirmingFull && (now - intakeStartTime >= INTAKE_CONFIRMATION_DURATION_NS)){
                    isFull = true;
                    isEmpty = false;
                    startedConfirmingFull = false;
                }
                break;
            case SHOOTING:
                if(!newExit && !exitTriggered && !startedConfirmingEmpty){
                    feedStartTime = System.nanoTime();
                    startedConfirmingEmpty = true;
                } else if(newExit && startedConfirmingEmpty){
                    feedStartTime = 0;
                    startedConfirmingEmpty = false;
                }

                if(startedConfirmingEmpty && (now - feedStartTime >= FEED_CONFIRMATION_DURATION_NS)){
                    isFull = false;
                    isEmpty = true;
                    startedConfirmingEmpty = false;
                }
                break;
            case OUTTAKING:
                if(!startedConfirmingEmpty && !newIntake && !intakeTriggered && (now - reverseStartTime >= REVERSE_RESET_DURATION_NS)) {
                    isEmpty = true;
                    isFull = false;
                    startedConfirmingEmpty = true;
                }
                break;
        }
        intakeTriggered = newIntake;
        exitTriggered = newExit;
    }

    public boolean isStorageFull() { return isFull; }
    public boolean isStorageEmpty() { return isEmpty; }
    public boolean isReadyToShoot() { return exitTriggered; }

    public void debug(Telemetry t){
        t.addData("intake cm", intakeSensor.getDistance(DistanceUnit.CM));
        t.addData("exit cm", exitSensor.getDistance(DistanceUnit.CM));
        t.addData("intake triggered", intakeTriggered);
        t.addData("exit triggered", exitTriggered);
        t.addData("is full", isStorageFull());
        t.addData("is empty", isStorageEmpty());
        t.addData("is ready to shoot", isReadyToShoot());
        t.addData("state", currentState);
        t.addData("Intake confirming start time", intakeStartTime);
        t.addData("Feed confirming start time", feedStartTime);
        t.addData("Blocker start duration", blockerStartTime);
    }
}