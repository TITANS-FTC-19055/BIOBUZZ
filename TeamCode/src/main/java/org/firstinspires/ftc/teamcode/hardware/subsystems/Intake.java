package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

@Configurable
public class Intake implements Updateable {

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

    public static double BLOCKER_BLOCK_POSITION = 0;
    public static double BLOCKER_RELEASE_POSITION = 0.24;

    public static double INTAKE_SENSOR_THRESHOLD_CM = 5.0;
    public static double EXIT_SENSOR_THRESHOLD_CM = 5.0;

    public static double FULL_CONFIRM_MS = 150;     // pollen on BOTH sensors
    public static double EMPTY_CONFIRM_MS = 300;    // pollen on NEITHER sensor
    public static double BLOCKER_OPEN_MS = 100;     // wait for the blocker before feeding

    private final DcMotor intake, transfer;
    private final DistanceSensor intakeSensor, exitSensor;
    private final Servo blocker;

    private final ElapsedTime fullTimer = new ElapsedTime();
    private final ElapsedTime emptyTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private State currentState = State.STOPPED;

    private boolean intakeTriggered, exitTriggered;
    private boolean isFull, isEmpty;

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

        blocker.setPosition(BLOCKER_BLOCK_POSITION);
    }

    public Command toggleIntaking(){
        return instant(() -> {
            if(currentState == State.STOPPED){
                activate().schedule();
            } else {
                stop().schedule();
            }
        });
    }

    public Command toggleReversing(){
        return instant(() -> {
            if(currentState == State.STOPPED){
                reverse().schedule();
            } else {
                stop().schedule();
            }
        });
    }

    public Command activate() {
        return infinite(() -> {
            run(State.INTAKING, INTAKE_POWER, TRANSFER_POWER);
        })
                .until(this::isStorageFull)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command reverse() {
        return infinite(() -> {
            run(State.OUTTAKING, REVERSE_INTAKE_POWER, REVERSE_TRANSFER_POWER);
        })
                .until(this::isStorageEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command feedToShooter() {
        return infinite(() -> {
            run(State.SHOOTING, INTAKE_POWER, TRANSFER_POWER);
        })
                .until(this::isStorageEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command continousFeedToShooter() {
        return infinite(() -> {
            run(State.SHOOTING, INTAKE_POWER, TRANSFER_POWER);
        })
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command stop() {
        return instant(this::halt).requiring(this);
    }

    private void run(State state, double intakePow, double transferPow) {
        if(currentState != state){
            currentState = state;
            stateTimer.reset();
            blocker.setPosition(state == State.SHOOTING
                    ? BLOCKER_RELEASE_POSITION
                    : BLOCKER_BLOCK_POSITION);
        }

        boolean blockerOpening = state == State.SHOOTING && stateTimer.milliseconds() < BLOCKER_OPEN_MS;

        intake.setPower(blockerOpening ? 0 : intakePow);
        transfer.setPower(blockerOpening ? 0 : transferPow);
    }

    private void halt() {
        intake.setPower(0);
        transfer.setPower(0);
        blocker.setPosition(BLOCKER_BLOCK_POSITION);
        currentState = State.STOPPED;
    }

    @Override
    public void update() {
        double intakeCm = intakeSensor.getDistance(DistanceUnit.CM);
        double exitCm = exitSensor.getDistance(DistanceUnit.CM);
        intakeTriggered = intakeCm < INTAKE_SENSOR_THRESHOLD_CM;
        exitTriggered = exitCm < EXIT_SENSOR_THRESHOLD_CM;

        if(!(intakeTriggered && exitTriggered)) fullTimer.reset();
        if(intakeTriggered || exitTriggered) emptyTimer.reset();

        isFull = fullTimer.milliseconds() > FULL_CONFIRM_MS;
        isEmpty = emptyTimer.milliseconds() > EMPTY_CONFIRM_MS;
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
    }
}