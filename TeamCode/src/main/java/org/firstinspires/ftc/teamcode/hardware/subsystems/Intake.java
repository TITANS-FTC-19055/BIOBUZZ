package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.HardwareConfig;

// Add @Config here (com.acmerobotics.dashboard.config.Config) if you use FTC Dashboard
public class Intake {

    public enum State {
        INTAKING,
        OUTTAKING,
        SHOOTING,
        STOPPED
    }

    // ---- Tunables ----
    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double REVERSE_INTAKE_POWER = -1.0;
    public static double REVERSE_TRANSFER_POWER = -1.0;

    // Hysteresis: triggers below DETECTION, releases only above RELEASE
    public static double DETECTION_THRESHOLD_CM = 2.0;
    public static double RELEASE_THRESHOLD_CM = 3.0;

    // Both sensors must stay free this long before we trust "ramp is empty"
    public static long EMPTY_CONFIRM_MS = 200;

    // Safety timeouts so a miscount can never lock the subsystem
    public static long FEED_TIMEOUT_MS = 4000;
    public static long REVERSE_TIMEOUT_MS = 3000;

    // ---- Constants ----
    private static final int MAX_CAPACITY = 4;
    private static final long MIN_READ_INTERVAL_NS = 5_000_000L; // 5 ms

    // ---- Hardware ----
    private final DcMotor intake, transfer;
    private final DistanceSensor intakeSensor, exitSensor;

    // ---- State ----
    private State currentState = State.STOPPED;
    private int elementCount = 0;

    // Cached (debounced) sensor values
    private boolean intakeTriggered = false;
    private boolean exitTriggered = false;
    private long lastReadNs = 0;

    private long stateStartNs = 0;
    private long emptySinceNs = 0; // 0 = sensors currently not both free

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

    // ---------------- Commands ----------------

    public Command activate() {
        return infinite(() -> {
            begin(State.INTAKING);
            updateSensors();
            setPower(INTAKE_POWER, TRANSFER_POWER);
        })
                .until(this::isFull)
                .unless(this::isFull)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command reverse() {
        return infinite(() -> {
            begin(State.OUTTAKING);
            updateSensors();
            setPower(REVERSE_INTAKE_POWER, REVERSE_TRANSFER_POWER);
        })
                .until(() -> isEmpty() || timedOut(REVERSE_TIMEOUT_MS))
                .unless(this::isEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command feedToShooter() {
        return infinite(() -> {
            begin(State.SHOOTING);
            updateSensors();
            setPower(INTAKE_POWER, TRANSFER_POWER);
        })
                .until(() -> isEmpty() || timedOut(FEED_TIMEOUT_MS))
                .unless(this::isEmpty)
                .setEnd(interrupted -> halt())
                .requiring(this);
    }

    public Command stop() {
        return instant(this::halt)
                .requiring(this);
    }

    // ---------------- Internals ----------------

    private void setPower(double intakePow, double transferPow) {
        intake.setPower(intakePow);
        transfer.setPower(transferPow);
    }

    /** Stops the motors and resets the state. Called on normal end AND on interruption. */
    private void halt() {
        setPower(0, 0);
        currentState = State.STOPPED;
        emptySinceNs = 0;
    }

    /**
     * Called at the top of every command body. On the first loop of a new command it
     * re-syncs the cached sensor values (so no false edge is seen) and starts the timeout clock.
     */
    private void begin(State newState) {
        if (currentState == newState) return;

        currentState = newState;
        long now = System.nanoTime();
        stateStartNs = now;
        emptySinceNs = 0;

        // Fresh read, but no edge processing
        intakeTriggered = readTriggered(intakeSensor, intakeTriggered);
        exitTriggered = readTriggered(exitSensor, exitTriggered);
        lastReadNs = now;
    }

    private boolean timedOut(long ms) {
        return currentState != State.STOPPED
                && (System.nanoTime() - stateStartNs) > ms * 1_000_000L;
    }

    /** Reads hardware at most once every MIN_READ_INTERVAL_NS, then updates the count. */
    private void updateSensors() {
        long now = System.nanoTime();
        if (now - lastReadNs < MIN_READ_INTERVAL_NS) return;
        lastReadNs = now;

        boolean newIntake = readTriggered(intakeSensor, intakeTriggered);
        boolean newExit = readTriggered(exitSensor, exitTriggered);

        processEdges(newIntake, newExit, now);

        intakeTriggered = newIntake;
        exitTriggered = newExit;
    }

    private boolean readTriggered(DistanceSensor sensor, boolean wasTriggered) {
        double d = sensor.getDistance(DistanceUnit.CM);
        // NaN compares false -> counts as "not triggered"
        return d < (wasTriggered ? RELEASE_THRESHOLD_CM : DETECTION_THRESHOLD_CM);
    }

    private void processEdges(boolean newIntake, boolean newExit, long nowNs) {
        switch (currentState) {
            case INTAKING:
                // Rising edge on intake -> a new piece entered
                if (newIntake && !intakeTriggered && elementCount < MAX_CAPACITY) {
                    elementCount++;
                }
                // Both ends covered -> ramp is full
                if (newIntake && newExit) {
                    elementCount = MAX_CAPACITY;
                }
                break;

            case SHOOTING:
                // Falling edge on exit -> a piece left toward the shooter
                if (!newExit && exitTriggered && elementCount > 0) {
                    elementCount--;
                }
                confirmEmpty(newIntake, newExit, nowNs);
                break;

            case OUTTAKING:
                // Falling edge on intake -> a piece was pushed back out
                if (!newIntake && intakeTriggered && elementCount > 0) {
                    elementCount--;
                }
                confirmEmpty(newIntake, newExit, nowNs);
                break;

            case STOPPED:
                break;
        }
    }

    /** Only zero the count if both sensors stay free for EMPTY_CONFIRM_MS. */
    private void confirmEmpty(boolean newIntake, boolean newExit, long nowNs) {
        if (newIntake || newExit) {
            emptySinceNs = 0;
            return;
        }
        if (emptySinceNs == 0) {
            emptySinceNs = nowNs;
        } else if (nowNs - emptySinceNs >= EMPTY_CONFIRM_MS * 1_000_000L) {
            elementCount = 0;
        }
    }

    // ---------------- Public queries ----------------

    public int getElementCount() {
        return elementCount;
    }

    public boolean isFull() {
        updateSensors();
        return elementCount >= MAX_CAPACITY;
    }

    public boolean isEmpty() {
        updateSensors();
        return elementCount == 0 && !intakeTriggered && !exitTriggered;
    }

    public boolean isReadyToShoot() {
        updateSensors();
        return exitTriggered;
    }

    public void setElementCount(int count) {
        this.elementCount = Math.max(0, Math.min(MAX_CAPACITY, count));
    }

    public boolean isRunning() {
        return currentState != State.STOPPED;
    }

    public State getState() {
        return currentState;
    }
}