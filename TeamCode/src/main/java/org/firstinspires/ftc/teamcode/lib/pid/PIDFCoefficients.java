package org.firstinspires.ftc.teamcode.lib.pid;

public class PIDFCoefficients {
    public double kP=0,kI=0,kD=0,kF=0;
    public PIDFCoefficients(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    public PIDFCoefficients(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
