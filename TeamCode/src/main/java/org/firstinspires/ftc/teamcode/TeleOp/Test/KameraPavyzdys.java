package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Kamera.Kamera.Kamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class KameraPavyzdys extends OpMode {

    Kamera kamera = new Kamera();
    @Override
    public void init() {
        kamera.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //Atnaujinti duomenis
        kamera.update();
        AprilTagDetection id20 = kamera.SpecID(20);
        telemetry.addData("id20 String", id20.toString());


    }
}
