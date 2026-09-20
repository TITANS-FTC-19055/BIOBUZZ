package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public enum IntakeState{
        ON(1), OFF(0), SPIT(-1);
        public final double val;
        IntakeState(double val){
            this.val=val;
        }

    }
}
