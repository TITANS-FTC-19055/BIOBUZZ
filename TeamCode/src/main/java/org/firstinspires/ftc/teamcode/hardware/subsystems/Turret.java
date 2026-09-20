package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.HardwareConfig;
import org.firstinspires.ftc.teamcode.lib.interfaces.Updateable;

public class Turret implements Updateable {
    private final Servo hrot1, hrot2;

    public Turret(@NonNull HardwareMap hwmap){
        hrot1 = hwmap.get(Servo.class, HardwareConfig.hrot1);
        hrot2 = hwmap.get(Servo.class, HardwareConfig.hrot2);
    }

    @Override
    public void update() {

    }
}
