package org.firstinspires.ftc.teamcode.hardware;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import androidx.annotation.NonNull;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.Constants.*;
import org.firstinspires.ftc.teamcode.hardware.pedro.Constants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretHRot;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Robot implements Updateable {
    private final Telemetry telemetry;
    private final Follower follower;

    private final Intake intake;
    private final Turret turret;
    private final TurretHRot hrot;

    public Alliance alliance = Alliance.RED;

    public Robot(@NonNull HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, OpModeStorage.autoEndPose);
    }

    public Robot(@NonNull HardwareMap hardwareMap, Telemetry telemetry, Pose startingPose) {
        this.telemetry = telemetry;
        follower = Constants.create(hardwareMap);
        if(startingPose != null && follower != null){
            follower.setPose(startingPose);
            alliance = determineAllianceFromPose(startingPose);
        }

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        hrot = new TurretHRot(hardwareMap);
    }

    public void drive(double forward, double strafe, double rotate){
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                rotate,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);
    }

    public Cell getClosestCell(){
        Pose currentPose = follower.pose();

        Cell audienceCell = (alliance == Alliance.RED)
                ? Cell.RED_AUDIENCE
                : Cell.BLUE_AUDIENCE;

        Cell scoringCell = (alliance == Alliance.RED)
                ? Cell.RED_SCORING
                : Cell.BLUE_SCORING;

        double distToAudience = currentPose.distance(audienceCell.pose);
        double distToScoring = currentPose.distance(scoringCell.pose);

        return (distToAudience < distToScoring) ? audienceCell : scoringCell;
    }

    public Alliance determineAllianceFromPose(@NonNull Pose startingPose){
        if(startingPose.x() >= 0 && startingPose.x() <= 72){
            return Alliance.RED;
        }
        else{
            return Alliance.BLUE;
        }

    }

    public Command shootSequence(double targetRpm){
        return instant(() -> Scheduler.schedule(
                turret.spinTo(targetRpm),
                sequential(
                        turret.waitUntilReady(),
                        turret.boost(0.15),
                        intake.collect(),
                        turret.boost(0.0)
                )
        ));
    }

    @Override
    public void update() {

        follower.update();
        telemetryData();
    }

    public void telemetryData(){
        telemetry.addData("X:", follower.pose().x());
        telemetry.addData("Y:", follower.pose().y());
        telemetry.addData("Heading:", follower.pose().heading());
        telemetry.addLine("--------------------------------------------------");
        telemetry.addData("Turret Angle:", hrot.getCurrentAngle());
        telemetry.addLine("--------------------------------------------------");
        telemetry.addData("Alliance:", getAlliance());
    }

    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }
    public Alliance getAlliance() {
        return alliance;
    }


}
