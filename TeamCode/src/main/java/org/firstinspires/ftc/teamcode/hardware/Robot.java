package org.firstinspires.ftc.teamcode.hardware;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.Constants.*;
import org.firstinspires.ftc.teamcode.hardware.pedro.Constants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Turret;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Robot implements Updateable {
    private final Follower follower;

    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;

    public Alliance alliance = Alliance.RED;
    public Pose closestCell = null;
    public double targetRpm = 0;

    public Robot(@NonNull HardwareMap hardwareMap){
        this(hardwareMap, OpModeStorage.autoEndPose);
    }

    public Robot(@NonNull HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.create(hardwareMap);
        if(startingPose != null && follower != null){
            follower.setPose(startingPose);
            alliance = determineAllianceFromPose(startingPose);
        }

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    public void drive(double forward, double strafe, double rotate){
        if(follower == null) return;
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                strafe,
                rotate,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);
    }


    public Command shootSequence(){
        return parallel(
                shooter.spin(this::calculateTargetRpm),
                turret.trackFieldPose(() -> getClosestCell().pose, follower::pose),
                sequential(
                        shooter.waitUntilReady(),
                        turret.waitUntilReady(),
                        intake.feedToShooter()
                )

        );
    }

    private Cell getClosestCell(){
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

    private double calculateTargetRpm(){
        Pose target = getClosestCell().pose;
        double distance = follower.pose().distance(target);

        return getRpmForDistance(distance);
    }

    private double getRpmForDistance(double distance){
        // Placeholder
        return 1500;
    }

    private Alliance determineAllianceFromPose(@NonNull Pose startingPose){
        return (startingPose.x() >= 0 && startingPose.x() <= 72)
                ? Alliance.RED
                : Alliance.BLUE;
    }

    @Override
    public void update() {
        if(follower != null) {
            follower.update();
        }
    }

    public void debug(Telemetry t){
        t.addData("X:", follower.pose().x());
        t.addData("Y:", follower.pose().y());
        t.addData("Heading:", follower.pose().heading());
        t.addLine("--------------------------------------------------");
        t.addData("Turret Angle:", turret.getCurrentAngle());
        t.addLine("--------------------------------------------------");
        t.addData("Alliance:", getAlliance());
        t.addData("Closest Cell:", getClosestCell());
    }

    public void setAlliance(Alliance alliance){
        this.alliance = alliance;
    }
    public Alliance getAlliance() {
        return alliance;
    }


}
