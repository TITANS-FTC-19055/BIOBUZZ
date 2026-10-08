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

    public static double SENSOR_THRESHOLD_CM = 2.0;

    private static final int MAX_CAPACITY = 4;
    private static final long MIN_READ_INTERVAL_NS = 5_000_000L; // 5 ms

    private final DcMotor intake, transfer;
    private final DistanceSensor intakeSensor, exitSensor;

    private State currentState = State.STOPPED;
    private int elementCount = 0;

    private boolean intakeTriggered = false;
    private boolean exitTriggered = false;
    private long lastReadNs = 0;

    public Intake(@NonNull HardwareMap hwmap) {
        intake = hwmap.get(DcMotor.class, HardwareConfig.intake);
        transfer = hwmap.get(DcMotor.class, HardwareConfig.transfer);

        intakeSensor = hwmap.get(DistanceSensor.class, HardwareConfig.intake_sensor);
        exitSensor = hwmap.get(DistanceSensor.class, HardwareConfig.exit_sensor);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Command activate() {
        return infinite(() -> {
            run(State.INTAKING, INTAKE_POWER, TRANSFER_POWER);
        })
                .until(this::isFull)
                .unless(this::isFull)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command reverse() {
        return infinite(() -> {
            run(State.OUTTAKING, REVERSE_INTAKE_POWER, REVERSE_TRANSFER_POWER);
        })
                .until(this::isEmpty)
                .unless(this::isEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command feedToShooter() {
        return infinite(() -> {
            run(State.SHOOTING, INTAKE_POWER, TRANSFER_POWER);
        })
                .until(this::isEmpty)
                .unless(this::isEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command stop() {
        return instant(this::halt).requiring(this);
    }

    private void run(State state, double intakePow, double transferPow) {
        currentState = state;
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
        if (now - lastReadNs < MIN_READ_INTERVAL_NS) return;
        lastReadNs = now;

        boolean newIntake = intakeSensor.getDistance(DistanceUnit.CM) < SENSOR_THRESHOLD_CM;
        boolean newExit   = exitSensor.getDistance(DistanceUnit.CM)   < SENSOR_THRESHOLD_CM;

        switch (currentState) {
            case INTAKING:
                if (newIntake && !intakeTriggered && elementCount < MAX_CAPACITY) elementCount++;
                if (newIntake && newExit) elementCount = MAX_CAPACITY;
                break;
            case SHOOTING:
                if (!newExit && exitTriggered && elementCount > 0) elementCount--;
                break;
            case OUTTAKING:
                if (!newIntake && intakeTriggered && elementCount > 0) elementCount--;
                break;
        }
        intakeTriggered = newIntake;
        exitTriggered = newExit;
    }

    public boolean isFull() { return elementCount >= MAX_CAPACITY; }
    public boolean isEmpty() { return elementCount == 0 && !intakeTriggered && !exitTriggered; }
    public boolean isReadyToShoot() { return exitTriggered; }
    public int getElementCount() { return elementCount; }
    public void setElementCount(int count) { this.elementCount = count; }

    public void debug(Telemetry t){
        t.addData("intake cm", intakeSensor.getDistance(DistanceUnit.CM));
        t.addData("exit cm", exitSensor.getDistance(DistanceUnit.CM));
        t.addData("element count", elementCount);
        t.addData("intake triggered", intakeTriggered);
        t.addData("exit triggered", exitTriggered);
        t.addData("is full", isFull());
        t.addData("is empty", isEmpty());
        t.addData("is ready to shoot", isReadyToShoot());
        t.addData("state", currentState);
    }
}