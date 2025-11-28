package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Kamera.AprilLibrary;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp (name = "MainTest")
public class MainTleOpTest extends LinearOpMode {
    Motor kP, kG, dP, dG; //kairÄ— priekis/galas, desinÄ— priekis/galas

    int KP=0,KG=0,DP=0,DG=0;
    DcMotorEx sm1,sm2; //PaÄ—mimas, iÅmetimas //0, 1, 2expansion hub
    DcMotor pad, pem;
    //AprilTag skirti dalykai
    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    // Start at ~70% power
    private double targetVelocity = MAX_TICKS_PER_SEC * 1;   ///derinsimes
    double x,y; //aprilTag x , y detection
    int id=0;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    //--------------------
    boolean prev = false;
    boolean motorOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Važiuoklės varikliai
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub
        kP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        /*kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub*/


        //Išmetimas/Paėmimas

        sm1 = hardwareMap.get(DcMotorEx.class, "svD");  // 0 lizdas expansion hub
        sm2 = hardwareMap.get(DcMotorEx.class, "svK");  // 1 lizdas expansion hub
        pad = hardwareMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub

        sm1.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
        sm2.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);

        initAprilTag();

        waitForStart();
        while (!isStopRequested()) {


            // VaÅ¾iuoklÄ—
            MecanumDrive drive = new MecanumDrive(kP, dP, kG, dG) ;

            if(!motorOn){ drive.driveRobotCentric(
                    gamepad1.left_stick_x * 0.85, // strafe/drift
                    -gamepad1.left_stick_y * 0.85, // priekis
                    gamepad1.right_stick_x * 0.85

            );
            }
            if (gamepad1.right_bumper) {
                pem.setPower(-0.5);
            } else if (!gamepad1.right_bumper) {
                pem.setPower(0);
            }
            if (gamepad1.dpad_up) {
                pad.setPower(0.5);
            }

            if (gamepad1.dpad_down) {
                pad.setPower(0);
            }

            /*telemetry.addData("Titas = blogas", x);
            telemetry.addData("Greitis ",targetVelocity);
            //double realtime_rpm1 = 6000*sm1.getPower();
            double realtime_rpm2 = 6000*abs(sm1.getPower());
            telemetry.addData("Y = ", y);
            if(gamepad1.circle)telemetry.update();*/


            if (gamepad1.square) {
                if (sm2.getPower() >= targetVelocity) {
                    gamepad1.rumble(500, 500, 600);
                }
                sm1.setPower(targetVelocity);
                sm2.setPower(targetVelocity);
                sleep(400);
                sm1.setPower(targetVelocity);
                sm2.setPower(targetVelocity);
                pad.setPower(0.4);
                pem.setPower(-0.5);
                sleep(900);
                sm1.setPower(0);
                sm2.setPower(0);
                pad.setPower(0);
                pem.setPower(0);
                drive.driveRobotCentric(
                        0,
                        0,
                        0
                );

            }


            //Taiklumo korekcija
            while (gamepad1.left_bumper)
            {
                double sp = 0.2; //Greitis
                telemetryAprilTag();
                telemetry.update();
                    ///Ieskoti tarp 20/25
                      if (x > 20 && x < 25) {
                        kP.set(sp);
                        dP.set(sp);
                        dG.set(sp);
                        kG.set(sp);
                        ///Šauti tarp
                        if (x < 20 || x > -20) {
                            break;
                        }
                    }
                      /// Ieškoti tarp -20
                      if (x < -20) {
                        kP.set(-sp);
                        dP.set(-sp);
                        dG.set(-sp);
                        kG.set(-sp);
                        if (x < 20 || x > -20) {
                            break;
                        }
                      }
                      /// Šauti
                    if (x < 20 && x > -20) {
                        kP.set(0);
                        dP.set(0);
                        dG.set(0);
                        kG.set(0);

                            if (sm2.getPower() >= targetVelocity) {
                                gamepad1.rumble(500, 500, 600);
                            }
                            sm1.setPower(targetVelocity);
                            sm2.setPower(targetVelocity);
                            sleep(400);
                            sm1.setPower(targetVelocity);
                            sm2.setPower(targetVelocity);
                            pad.setPower(0.4);
                            pem.setPower(-0.5);
                            sleep(900);
                            sm1.setPower(0);
                            sm2.setPower(0);
                            pad.setPower(0);
                            pem.setPower(0);
                            drive.driveRobotCentric(
                                    0,
                                    0,
                                    0
                            );


                    }
                    id=0;
            }



        }
    }

    private void initAprilTag() {



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
    private void telemetryAprilTag() {

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
