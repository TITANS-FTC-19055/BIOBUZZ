package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class OpModeStorage {
    static PoseFactory factory = PoseFactory.degrees();
    public static Pose autoEndPose = factory.of(0.0, 0.0, 90);
}
