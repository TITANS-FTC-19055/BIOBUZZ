package org.firstinspires.ftc.teamcode.lib.pid;

public class VelocityPID {
    public double kP=0, kI=0, kD=0, kF=0;
    public double targetVelocity, sumError, lastError, lastPosition,currentVelocity;
    public long lastTime;
    public VelocityPID(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    public VelocityPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setTargetVelocity(double targetVelocity) {
        resetPid();
        this.targetVelocity = targetVelocity;
    }

    public void resetPid() {
        sumError = 0;
        lastError = 0;
        lastTime = 0;
    }

    public double update(double position){
        long currentTime = System.nanoTime();

        double deltaTime = (currentTime-lastTime)*1e-9;

        currentVelocity = (position-lastPosition)/deltaTime;
        double error = targetVelocity-currentVelocity; // velocity error
        double delta_e = lastError - error;
        lastError = error;

        if (lastTime==0){
            lastTime = currentTime;
            lastPosition = position;
            return 0;
        }

        sumError += error*deltaTime;


        lastTime = currentTime;
        lastPosition = position;

        return (error*kP +
                sumError*kI +
                (delta_e)/deltaTime*kD
                +kF);
    }
}
