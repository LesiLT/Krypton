package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AutoV1_su_kamera extends LinearOpMode {

    //Rasto parinkimas
    int n = 10; //Rašto pasirinkimas
    int rX = 0;
    DcMotor kP,kG,dP,dG;
    DcMotor sm1,sm2,pm;
    CRServo P1S,P2S;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub

        //Išmetimas/Paėmimas
        sm1 = hardwareMap.get(DcMotor.class, "sm1");
        sm2 = hardwareMap.get(DcMotor.class, "sm2");
        pm = hardwareMap.get(DcMotor.class, "pm");


        //Padavimo servai
        P1S = hardwareMap.get(CRServo.class, "P1S");
        P2S = hardwareMap.get(CRServo.class, "P2S");

        Pose2d beginPose = new Pose2d(0, 0, Math.toDegrees(315));//Pradine pozicija 45 laipsniai nuo sienos
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose); //Odometrijos iskvietimas

        initAprilTag();

        waitForStart();

        //kelio parinkimas
        //21: artimiausias GPP
        //22: vidurinis PGP
        //23: tolimiausias PPG
        if (rX == 21) {
            n = 10;
        } else if (rX == 22) {
            n = 20;
        } else if (rX == 23) {
            n = 30;
        }

        Actions.runBlocking(drive.actionBuilder(beginPose)
                .stopAndAdd(new sovimas(sm1,sm2, P1S,P2S, 0.5))
                .turn(0.3333) //Atsisukti į apriltag
                .lineToXLinearHeading(n, Math.toDegrees(90)) //važiutoi iki paėmimo ir atsisukti
                //Sukti paėmimą ir paiimti
                .stopAndAdd(new sovimas(sm1,sm2, P1S,P2S, 0.5))
                .strafeToLinearHeading(new Vector2d(0, 0), Math.toDegrees(0.33333)) //Grįžti atgal
                .stopAndAdd(new sovimas(sm1,sm2, P1S,P2S, 0.5))
                .build())
        ;

        visionPortal.close();
    }


    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
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

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()


    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

            rX = detection.id;

        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()


    public class paemimas implements Action {
        DcMotor kP, kG, dP, dG;
        DcMotor pm;
        double sp; //greitis
        CRServo P1S, P2S; // paemimo servai

        public paemimas (DcMotor pm, CRServo P1S, CRServo P2S, double sp, DcMotor kP, DcMotor kG, DcMotor dP, DcMotor dG) {
            this.kG = kG;
            this.kP = kP;
            this.dP = dP;
            this.dG = dG;
            this.P2S = P2S;
            this.P1S = P1S;
            this.pm = pm;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            resetRuntime();
            while(getRuntime() < 5){
                kG.setPower(sp);
                kP.setPower(sp);
                dP.setPower(sp);
                dG.setPower(sp);
                pm.setPower(sp);
                P1S.setPower(sp);
                P2S.setPower(sp);
            }

            return false;
        }
    }
    public class sovimas implements Action{
        DcMotor sm1,sm2;
        CRServo P1S, P2S;
        double sp;
        public sovimas(DcMotor sm1,DcMotor sm2, CRServo P1S,CRServo P2S,double sp){
            this.sm1 = sm1;
            this.sm2 = sm2;
            this.P1S = P1S;
            this.P2S = P2S;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            resetRuntime();
            int n = 0;
            while(getRuntime() < 2){
                sm1.setPower(sp);
                sm2.setPower(sp);
                P1S.setPower(sp);
                P2S.setPower(sp);
                resetRuntime();
                n++;
                if (n == 3) break;
            }

            return false;
        }
    }
}