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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "Auto+2Blue0")
public class Auto2MO extends LinearOpMode {

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


        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(-27)
                .stopAndAdd(new šauti(sm1, sm2, pad, pem, 0.9))
                        .lineToX(-33)
                        .stopAndAdd(new suktis(kP, kG, dG, dP, 1))
                .build());
        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeTo(new Vector2d(0, 5))
                        .build()
        );

                pem.setPower(-0.6);
        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(25)
                        .lineToX(20)
                        .stopAndAdd(new suktis(kP, kG, dG, dP, -1))

                .build()
        );
        Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                .strafeTo(new Vector2d(0, -10))
                .stopAndAdd(new šauti(sm1, sm2, pad, pem, 0.9))
                .build());



    }


    public class suktis implements Action{
        DcMotor kP, kG, dP, dG;

        int k;

        public suktis (DcMotor kP, DcMotor kG, DcMotor dG, DcMotor dP,  int k){
            this.dG = dG;
            this.dP = dP;
            this.kG = kG;
            this.kP = kP;
            this.k = k;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pem.setPower(0);
            sm1.setPower(0);
            sm2.setPower(0);
            pad.setPower(0);
            kP.setPower(-0.3 * k);
            dG.setPower(0.3 * k);
            dP.setPower(0.3 * k);
            kG.setPower(-0.3 * k);
            sleep(350);
            kP.setPower(-0);
            dG.setPower(0);
            dP.setPower(0);
            kG.setPower(-0);

            return false;
        }
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