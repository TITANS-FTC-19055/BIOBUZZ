package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants.RobotState;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Robot implements Updateable {
    public Intake intake;
    public Drivetrain drivetrain;
    public Turret turret;
    public RobotState state = RobotState.IDLE;
    public Robot(@NonNull HardwareMap hardwareMap) {
        intake= new Intake(hardwareMap);
        drivetrain= new Drivetrain(hardwareMap,true);
        turret= new Turret(hardwareMap);
    }

    @Override
    public void update() {
        intake.update();
        turret.update();
    }
    public void setState(RobotState state){
        if(this.state == state) this.state = RobotState.IDLE;
        else this.state = state;
    }

}
