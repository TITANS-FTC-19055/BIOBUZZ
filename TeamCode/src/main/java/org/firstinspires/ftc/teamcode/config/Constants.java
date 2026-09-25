package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class Constants {
    public static final Pose RED_SCORING_CELL = new Pose(0.0, 0.0);
    public static final Pose RED_AUDIENCE_CELL = new Pose(0.0, 0.0);
    public static final Pose BLUE_SCORING_CELL = new Pose(0.0, 0.0);
    public static final Pose BLUE_AUDIENCE_CELL = new Pose(0.0, 0.0);

    public enum Cell {
        RED_SCORING(RED_SCORING_CELL), RED_AUDIENCE(RED_AUDIENCE_CELL), BLUE_SCORING(BLUE_SCORING_CELL), BLUE_AUDIENCE(BLUE_AUDIENCE_CELL);
        public final Pose pose;
        Cell(Pose pose) {
            this.pose = pose;
        }
    }

    public enum Alliance{
        RED, BLUE;
    }

}
