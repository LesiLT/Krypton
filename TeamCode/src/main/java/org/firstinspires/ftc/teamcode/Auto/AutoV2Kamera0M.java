package org.firstinspires.ftc.teamcode.Auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "Auto Mėlynas")
public class AutoV2Kamera0M extends LinearOpMode {

    DcMotor kP,kG,dP,dG;
    DcMotor sm1,sm2;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
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

        Pose2d beginPose = new Pose2d(0, 0, Math.toDegrees(315));//Pradine pozicija 45 laipsniai nuo sienos
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose); //Odometrijos iskvietimas

        Actions.runBlocking(drive.actionBuilder(beginPose)
                .stopAndAdd(new šauti(sm1, sm2, 0.5))
                .lineToX(10)
                .build()
        );



    }

    public class šauti implements Action {
        DcMotor sm1, sm2;

        Servo P1, p2;


        double sp;
        public šauti(DcMotor sm1, DcMotor sm2, double sp) {
        this.sm1 = sm1;
        this.sm2 = sm2;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            sm1.setPower(sp);
            sm2.setPower(sp);

            return false;
        }
    }
}
