package org.firstinspires.ftc.teamcode.Mechanizmai;

import static android.os.SystemClock.sleep;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Kamera.AprilLibrary;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


//Dabar kuriant nauja Java Klase gali i si kreiptis su sitomis kodo linijomis:
//public HardwareInit hw;
//hw = new HardwareInit();
//hw.initializeHardware(hardwareMap);
//taip pat is naujo nereikia aprasyti Motoru, servu ar Dcmotoru, kreiptis su hw.(variable_name), pvz: hw.kP, hw.sm1, hw.pak0

//Zemiau surasyti visi metodai
public class HardwareInit_Metodai {

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    
    public Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas
    public DcMotorEx sm1, sm2; //Paėmimas, išmetimas //0, 1, 2expansion hub
    public DcMotor pad, pem;
    public Servo pak0, pak1; //Pakelimo servas Eh = 0, Eh 1

    public Servo kamp;

    public int id=0;
    public static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    public static final int MAX_RPM = 6000;
    public static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    // Start at ~70% power
    public double targetVelocity = MAX_TICKS_PER_SEC * 1;   ///derinsimes
    public double x,y; //aprilTag x , y detection

    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;

    public HardwareInit_Metodai(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initializeHardware();
        initAprilTag();
    }

    //Hardware pajungimo metodas-------------------------------------------------------------
    public void initializeHardware()
    {
        //Važiuoklės varikliai
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub

        /*kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub*/

        kP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //Išmetimas/Paėmimas

        sm1 = hardwareMap.get(DcMotorEx.class, "svD");  // 0 lizdas expansion hub
        sm2 = hardwareMap.get(DcMotorEx.class, "svK");  // 1 lizdas expansion hub
        pad = hardwareMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub

        /// Pak4limas
        pak0 = hardwareMap.get(Servo.class, "pak0");
        pak1 = hardwareMap.get(Servo.class, "pak1");
        kamp = hardwareMap.get(Servo.class, "kamp");

        pak0.setDirection(Servo.Direction.REVERSE);
        pak1.setDirection(Servo.Direction.REVERSE);
        pad.setDirection(DcMotor.Direction.REVERSE);
    }
    //Surinkimo metodai------------------------------------------------------------------------
    public void paemimas(){
        pem.setPower(-0.5);
        pad.setPower(0.01);
    }
    public void atgal(){
        pem.setPower(0.5);
    }
    public void stop(){
        pem.setPower(0);
    }

    //Saudykles metodai--------------------------------------------------------------------------

    public void ugnis() {
        sm1.setPower(targetVelocity);
        sm2.setPower(targetVelocity);
        sleep(400);
        sm1.setPower(targetVelocity);
        sm2.setPower(targetVelocity);
        pad.setPower(0.55);
        pem.setPower(-0.5);
        sleep(900);
        sm1.setPower(0);
        sm2.setPower(0);
        pad.setPower(0);
        pem.setPower(0);
    }
    public void atgal1(){
        pem.setPower(0.7);
        pad.setPower(-0.7);
        sm1.setPower(-0.6);
        sm2.setPower(-0.6);
    }

    public void atgal2()
    {
        pem.setPower(-0.7);
        // pad.setPower(-0.7);
        // sm1.setPower(-0.6);
        // sm2.setPower(-0.6);
    }
    public void atgal0(){
        pem.setPower(0);
        pad.setPower(-0);
        sm1.setPower(-0);
        sm2.setPower(-0);
    }

    //Kiti metodai-------------------------------------------------------------------------------

    public double posukis(int degrees)
    {
        return Math.toRadians(degrees);
    }

    public void initAprilTag() {



        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilLibrary.getSmallLibrary())
//
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //    .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {

                x = detection.ftcPose.x;
                y = detection.ftcPose.y;
                id = detection.id;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("Y %6.1f (cm)",  detection.ftcPose.y));
                telemetry.addLine(String.format("X %6.1f (deg)", detection.ftcPose.pitch));
                telemetry.addLine(String.format("Z %6.1f (cm)", detection.ftcPose.range));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center   (pixels)", detection.center.x));
            }

        }   // end for() loop




    }   // end method telemetryAprilTag()

}
