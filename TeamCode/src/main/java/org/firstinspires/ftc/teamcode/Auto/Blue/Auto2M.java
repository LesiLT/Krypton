package org.firstinspires.ftc.teamcode.Auto.Blue;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "Auto+2Blue")
public class Auto2M extends LinearOpMode {

    DcMotor kP, kG, dP, dG;
    DcMotorEx sm1,sm2; //PaÄ—mimas, iÅmetimas //0, 1, 2expansion hub
    DcMotor pad, pem;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;



    @Override
    public void runOpMode() throws InterruptedException {

        kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub
        kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Išmetimas/Paėmimas
        sm1 = hardwareMap.get(DcMotorEx.class, "svD");  // 0 lizdas expansion hub
        sm2 = hardwareMap.get(DcMotorEx.class, "svK");  // 1 lizdas expansion hub
        pad = hardwareMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub
        sm2.setDirection(DcMotorSimple.Direction.REVERSE);
        //Kr1pton?5


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        //Atsitraukti
        kP.setPower(-0.15);
        dG.setPower(-0.15);
        dP.setPower(-0.15);
        kG.setPower(-0.15);
        sleep(200);
        kP.setPower(-0.4);
        dG.setPower(-0.4);
        dP.setPower(-0.4);
        kG.setPower(-0.4);
        sleep(860);
        kP.setPower(0);
        dG.setPower(-0);
        dP.setPower(-0);
        kG.setPower(0);
        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                //    .lineToX(-10)
                .stopAndAdd(new šauti(sm1, sm2, pad, pem, 0.9))
                .build());
        //Posukis
        pem.setPower(0);
        sm1.setPower(0);
        sm2.setPower(0);
        pad.setPower(0);
        kP.setPower(-0.3);
        dG.setPower(0.3);
        dP.setPower(0.3);
        kG.setPower(-0.3);
        sleep(500);
        //Strafe
        kP.setPower(-0.3);
        dG.setPower(-0.3);
        dP.setPower(0.3);
        kG.setPower(0.3);
        sleep(1750);
        //Paimti
        kP.setPower(0.35);
        dG.setPower(0.35);
        dP.setPower(0.35);
        kG.setPower(0.35);
        pem.setPower(-0.6);
        sleep(800);
        kP.setPower(0);
        dG.setPower(0);
        dP.setPower(0);
        kG.setPower(0);
        pem.setPower(-0.6);
        sleep(100);
        //Atsitraukti
        kP.setPower(0.35);
        dG.setPower(0.35);
        dP.setPower(-0.35);
        kG.setPower(-0.35);
        pem.setPower(-0);
        sleep(1650);
        kP.setPower(0.3);
        dG.setPower(-0.3);
        dP.setPower(-0.3);
        kG.setPower(0.3);
        sleep(500);
        kP.setPower(-0.25);
        dG.setPower(-0.25);
        dP.setPower(-0.25);
        kG.setPower(-0.25);
        sleep(400);
        kP.setPower(-0);
        dG.setPower(-0);
        dP.setPower(-0);
        kG.setPower(-0);
        //Strafe šovimui
        kP.setPower(0.35);
        dG.setPower(0.35);
        dP.setPower(-0.35);
        kG.setPower(-0.35);
        pem.setPower(-0.6);
        sleep(800);
        //Atsitraukti
        kP.setPower(-0.3);
        dG.setPower(-0.3);
        dP.setPower(-0.3);
        kG.setPower(-0.3);
        sleep(400);
        kP.setPower(-0);
        dG.setPower(-0);
        dP.setPower(-0);
        kG.setPower(-0);
        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                //    .lineToX(-10)
                .stopAndAdd(new šauti(sm1, sm2, pad, pem, 0.9))
                .build());
        //Strafe
        kP.setPower(-0.3);
        dG.setPower(-0.3);
        dP.setPower(0.3);
        kG.setPower(0.3);
        sleep(800);
        pem.setPower(0);
        sm1.setPower(0);
        sm2.setPower(0);
        pad.setPower(0);
        kP.setPower(-0.3);
        dG.setPower(-0.3);
        dP.setPower(0.3);
        kG.setPower(0.3);

    }


    public class šauti implements Action {
        DcMotorEx sm1,sm2; //PaÄ—mimas, iÅmetimas //0, 1, 2expansion hub
        DcMotor pad, pem;
        double sp;

        public šauti(DcMotorEx sm1, DcMotorEx sm2, DcMotor pad, DcMotor pem, double sp) {
            this.sm1 = sm1;
            this.sm2 = sm2;
            this.pad = pad;
            this.pem = pem;
            this.sp = sp;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            sm1.setPower(sp);
            sm2.setPower(-sp);
            sleep(800);
            sm1.setPower(sp);
            sm2.setPower(-sp);
            pad.setPower(0.5);
            pem.setPower(-0.5);
            sleep(1500);

            return false;

        }
    }
}