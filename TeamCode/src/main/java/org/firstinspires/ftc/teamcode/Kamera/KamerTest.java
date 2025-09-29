package org.firstinspires.ftc.teamcode.Kamera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class KamerTest extends OpMode {
    TrysKvadratai2 trysKvadratai;
    private VisionPortal visionPortal;
    @Override
    public void init() {
    trysKvadratai = new TrysKvadratai2();
    visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), trysKvadratai);
    }

    @Override
    public void loop() {

    }
}
