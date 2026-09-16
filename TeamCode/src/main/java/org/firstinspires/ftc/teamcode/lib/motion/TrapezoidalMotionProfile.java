package org.firstinspires.ftc.teamcode.lib.motion;

import android.util.Pair;

import com.qualcomm.robotcore.util.Range;

@SuppressWarnings("unused")
public class TrapezoidalMotionProfile {
    double maxAccel, maxVel;
    double accelTime = 0;
    boolean decelerate = false;
    double initialSpeed = 0;

    public TrapezoidalMotionProfile(double maxAccel, double maxVel, double initialVel){
        this.maxAccel =  maxAccel;
        this.maxVel = maxVel;
        this.initialSpeed = initialVel;

        this.accelTime = (maxVel - initialVel) / maxAccel;
    }

    public void regenerateProfile(double maxAccel, double maxVel, double initialVel){
        this.initialSpeed = initialVel;
        this.accelTime = (maxVel - initialVel) / maxAccel;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    // immediately starts decelerating
    public void toggleDeceleration(boolean decelerate, double currentSpeed){
        if(decelerate) {
            this.decelerate = true;
            this.initialSpeed = currentSpeed;
            this.accelTime = currentSpeed / maxAccel;
        } else {
            this.decelerate = false;
            regenerateProfile(this.maxAccel, this.maxVel, currentSpeed);
        }
    }

    // returns a pair of <plannedVel, plannedAccel>
    public Pair<Double, Double> get(double time) {
        if (!decelerate) {
            int isAccelerating = 1;

            if (time >= accelTime) {
                isAccelerating = 0;
            }

            return new Pair<>(Range.clip(time * maxAccel, -maxVel, maxVel), isAccelerating * maxAccel);
        } else {
            if(time >= accelTime){
                return new Pair<>(0d, 0d);
            }
            return new Pair<>(Range.clip(initialSpeed - time * maxAccel, - maxVel, maxVel), - maxAccel);
        }
    }
}
