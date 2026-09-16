package org.firstinspires.ftc.teamcode.lib.pid;

public class MultiplePIDF {
    public static final int max_length=10;

    public PIDFCoefficients[] pidfCoefficients = new PIDFCoefficients[max_length] ;
    public PIDFCoefficients currentPidCoefficients;
    public double targetPos = 0, lastError = 0, sumError = 0, error=0;
    public int coef_cont = 0;
    public long lastTime;
    public int current_pid_cont=0;

    public MultiplePIDF(double kP, double kI, double kD, double kF){
        clearPidCoefficients();
        pidfCoefficients[0] = new PIDFCoefficients(kP,kI,kD,kF);

        currentPidCoefficients = pidfCoefficients[0];


    }
    public MultiplePIDF(double kP, double kI, double kD){
        clearPidCoefficients();
        pidfCoefficients[0] = new PIDFCoefficients(kP,kI,kD);

        currentPidCoefficients = pidfCoefficients[0];
    }





    public void clearPidCoefficients(){
        for(int i=0;i<max_length;i++){
            pidfCoefficients[i] = new PIDFCoefficients(0,0,0,0);
            coef_cont = 0;
        }
    }
    public void addPidCoefficients(double kP, double kI, double kD, double kF){
        if (coef_cont+1<max_length)
            pidfCoefficients[++coef_cont] = new PIDFCoefficients(kP,kI,kD,kF);
    }
    public void addPidCoefficients(double kP, double kI, double kD){
        if (coef_cont+1<max_length)
            pidfCoefficients[++coef_cont] = new PIDFCoefficients(kP,kI,kD);

    }
    public void changeExistentPidCoefficients(int coef, double kP, double kI, double kD, double kF){
        if (coef<0||coef>max_length-1)
            return;
        pidfCoefficients[coef] = new PIDFCoefficients(kP,kI,kD,kF);

    }
    public void changeExistentPidCoefficients(int coef, double kP, double kI, double kD){
        if (coef<0||coef>max_length-1)
            return;
        pidfCoefficients[coef] = new PIDFCoefficients(kP,kI,kD);

    }
    public void switchPid(int cont){
        resetPid();
        currentPidCoefficients = pidfCoefficients[cont];
        current_pid_cont = cont;
    }

    public int whichPid(){
        return current_pid_cont;
    }


    public void resetPid(){
        sumError = 0;
        lastTime = System.nanoTime();
        lastError = 0;
        error = 0;
    }

    public void setTargetPosition(double target){
        resetPid();
        targetPos = target;
    }
    public void setTargetWithoutResetting(double target){
        targetPos = target;
    }

    public double getError(){
        return error;
    }

    public double update(double currentTicks){
        long currentTime = System.nanoTime();
        error = targetPos - currentTicks;
        double delta_e = lastError - error;
        double deltaTime = (currentTime-lastTime)*1e-9;

        sumError += error*deltaTime;
        lastError = error;
        lastTime = currentTime;

        return (error * currentPidCoefficients.kP +
                sumError * currentPidCoefficients.kI +
                delta_e/deltaTime * currentPidCoefficients.kD +
                currentPidCoefficients.kF);
    }
}