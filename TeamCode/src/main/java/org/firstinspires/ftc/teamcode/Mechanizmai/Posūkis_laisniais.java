package org.firstinspires.ftc.teamcode.Mechanizmai;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Posūkis_laisniais {
    double degrees;


    public void init(HardwareMap hardwareMap) {
    }
    public double posukis(double degrees)
    {
        return Math.toRadians(degrees);
    }
}
