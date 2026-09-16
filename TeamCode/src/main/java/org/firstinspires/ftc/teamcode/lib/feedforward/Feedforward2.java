package org.firstinspires.ftc.teamcode.lib.feedforward;

@SuppressWarnings("unused")
public class Feedforward2 {
    public double kV = 0, kA = 0, kStatic = 0;

    public Feedforward2(double kV, double kA, double kStatic ){
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }

    public double update(double profiledVel, double profiledAccel){
        return profiledVel * kV + profiledAccel * kA + kStatic * Math.signum(profiledVel);
    }
}
