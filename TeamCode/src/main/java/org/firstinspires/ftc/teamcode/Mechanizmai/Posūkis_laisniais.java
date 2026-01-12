package org.firstinspires.ftc.teamcode.Mechanizmai;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
public class Posūkis_laisniais {
    double degrees;


    public Posūkis_laisniais(HardwareMap hardwareMap) {
    }
    public double posukis(double degrees)
    {
         Math.toRadians(degrees);
        return degrees;
    }
}
