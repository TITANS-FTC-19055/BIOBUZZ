package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Robot implements Updateable {
    public Intake intake;
    public Drivetrain drivetrain;
    public Turret turret;
    public ROBOT_STATES state = ROBOT_STATES.IDLE;
    public Robot(@NonNull HardwareMap hardwareMap) {
        intake= new Intake(hardwareMap);
        drivetrain= new Drivetrain(hardwareMap,true);
        turret= new Turret(hardwareMap);
    }

    @Override
    public void update() {
        intake.update();
        turret.update();
        switch(state){
            case IDLE:
                turret.block.setPosition(0);
                intake.setState(RobotConstants.IntakeState.OFF);
                turret.setRotation(Turret.shooting_states.OFF);
                break;
            case COLLECTING:
                turret.block.setPosition(0);
                intake.setState(RobotConstants.IntakeState.ON);
                break;
            case SHOOTING:
                turret.block.setPosition(1);
                turret.setRotation(Turret.shooting_states.SHOOTING_FAR);
                intake.setState(RobotConstants.IntakeState.ON);
                break;
        }
    }
    public enum ROBOT_STATES{
        IDLE,
        COLLECTING,
        SHOOTING;

    }
    public void setState(ROBOT_STATES stateb){
        if(state==stateb)state = ROBOT_STATES.IDLE;
        else state = stateb;
    }

}
