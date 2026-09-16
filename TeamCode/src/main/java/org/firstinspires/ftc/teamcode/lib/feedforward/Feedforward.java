package org.firstinspires.ftc.teamcode.lib.feedforward;

import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

@SuppressWarnings("unused")
public class Feedforward {
    public double kV = 0, kA = 0, kStatic = 0, kP=0;
    public double dt=0, profiledVelocity=0, profiledAcceleration=0, currentVelocity=0, targetVelocity=0, maxAcceleration=0,  duration=0, lastPosition=0, increasing=1, acceleration=0,lastVelocity=0;
    public long lastTime=0,startTime=0;
    public boolean kp_disabled =false;
    public int index = 0;
    public int list_size = 10;
    public double velocitySum = 0;
    public double[] velocityList = new double[list_size];

    public Feedforward(double kP,double kV, double kA, double kStatic ){
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        reset();
        resetList();

    }
    public Feedforward(double kP,double kV, double kA, double kStatic, double maxAcceleration ){
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.maxAcceleration = maxAcceleration;
        reset();
        resetList();
    }

    public void setTarget(double target){
        reset();
        duration = Math.abs((target-targetVelocity))/maxAcceleration;
        startTime = System.nanoTime();
        if (target>=targetVelocity) increasing = 1;
        else increasing = -1;
        targetVelocity = target;
        profiledVelocity = velocitySum;

    }
    public void reset(){
        dt=0; profiledVelocity=0; profiledAcceleration=0; currentVelocity=0; lastTime=0; startTime=0; duration=0; lastPosition=0; acceleration=0; lastVelocity=0;
        resetList();
    }

    public double update(double currentPosition){
        long currentTime = System.nanoTime();
        double error;
        dt = (currentTime-lastTime)*1e-9;
        currentVelocity = (currentPosition-lastPosition)/dt;

        velocitySum -= velocityList[index];
        velocityList[index] = currentVelocity;
        velocitySum += velocityList[index++];
        if (index>list_size-1) index = 0;

        acceleration = (lastVelocity-currentVelocity)/dt;


        if ((profiledVelocity < targetVelocity &&increasing == 1) || (profiledVelocity > targetVelocity && increasing == -1))
            profiledVelocity += dt * maxAcceleration * increasing;
        else profiledVelocity = targetVelocity;

        error = profiledVelocity-currentVelocity; // for kP

        if ((currentTime-startTime)*1e-9>duration){
            profiledAcceleration = 0;
        }
        else {
            profiledAcceleration = maxAcceleration*increasing;
        }
        lastTime = currentTime;
        lastVelocity = currentVelocity;
        lastPosition = currentPosition;
        //return error*kP + targetVelocity * kV + profiledAcceleration * kA + kStatic;
        return Range.clip(error* (kp_disabled?0:kP) + targetVelocity*kV + kStatic + profiledAcceleration*kA,-1,1);
    }
    private void resetList(){
        Arrays.fill(velocityList, 0);
        velocitySum = 0;
    }




}
