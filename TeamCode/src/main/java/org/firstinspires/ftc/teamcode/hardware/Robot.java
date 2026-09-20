package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Robot implements Updateable {
    public final Drivetrain drivetrain;
    public final Intake intake;


    public Robot(@NonNull HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap, true);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void update() {
        intake.update();
    }
}
