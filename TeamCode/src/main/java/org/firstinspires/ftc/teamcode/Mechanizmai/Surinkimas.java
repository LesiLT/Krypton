package org.firstinspires.ftc.teamcode.Mechanizmai;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Surinkimas {
    DcMotor pem;
    public void init(HardwareMap hardwareMap) {
        pem = hardwareMap.get(DcMotor.class, "pem");

    }
    public void paemimas(){
        pem.setPower(-0.5);
    }
    public void atgal(){
        pem.setPower(0.5);
    }
    public void stop(){
        pem.setPower(0);
    }
}
