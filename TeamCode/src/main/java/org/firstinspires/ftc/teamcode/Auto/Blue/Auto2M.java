package org.firstinspires.ftc.teamcode.Auto.Blue;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/// O = odometrija
@Autonomous (name = "AutoBlue")
public class Auto2M extends LinearOpMode {

    DcMotor kP, kG, dP, dG;
    DcMotorEx sm1,sm2; //PaÄ—mimas, iÅmetimas //0, 1, 2expansion hub
    DcMotor pad, pem;
    Servo kamp;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;



    @Override
    public void runOpMode() throws InterruptedException {

        kamp = hardwareMap.get(Servo.class, "kamp");


        kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub
        kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        kamp.setPosition(0.25);


        /// Atsitraukti, pasisukti
        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                .turn(Math.PI/2)
                .build());
        /// Strafe


    }





}