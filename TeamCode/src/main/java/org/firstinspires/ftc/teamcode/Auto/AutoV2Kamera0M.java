package org.firstinspires.ftc.teamcode.Auto;


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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "Auto MėlynasDidysis")
public class AutoV2Kamera0M extends LinearOpMode {

    DcMotor kP, kG, dP, dG;
    DcMotor sm1, sm2;
    CRServo P1S, P2S;
    Servo pm;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    int Arti_Toli_ToliToli = 0;
    int KarD = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub

        //Išmetimas/Paėmimas
        sm1 = hardwareMap.get(DcMotor.class, "sm1");
        sm2 = hardwareMap.get(DcMotor.class, "sm2");
        P1S = hardwareMap.get(CRServo.class, "P1S");
        P2S = hardwareMap.get(CRServo.class, "P2S");
        pm = hardwareMap.get(Servo.class, "pm");

        while (!isStarted()) {
            if (gamepad1.dpad_up) {
                telemetry.addData("Atstumas", "Arti");
                Arti_Toli_ToliToli = 1;
                telemetry.update();
            } else if (gamepad1.dpad_left) {
                telemetry.addData("Atstumas", "Toli");
                Arti_Toli_ToliToli = 2;
                telemetry.update();
            } else if (gamepad1.dpad_down) {
                telemetry.addData("Atstumas", "Labai Toli");
                Arti_Toli_ToliToli = 3;
                telemetry.update();
            }

            if (Arti_Toli_ToliToli == 1) {
                if (gamepad1.right_bumper) {
                    telemetry.addData("Atstumas", "Arti");
                    telemetry.addData("Pusė", "Dešinė");
                    KarD = 2;
                    telemetry.update();
                } else if (gamepad1.left_bumper) {
                    telemetry.addData("Atstumas", "Arti");
                    telemetry.addData("Pusė", "Kairė");
                    KarD = 1;
                    telemetry.update();
                }
            } else if (Arti_Toli_ToliToli == 2) {
                if (gamepad1.right_bumper) {
                    telemetry.addData("Atstumas", "Toli");
                    telemetry.addData("Pusė", "Dešinė");
                    KarD = 2;
                    telemetry.update();
                } else if (gamepad1.left_bumper) {
                    telemetry.addData("Atstumas", "Toli");
                    telemetry.addData("Pusė", "Kairė");
                    KarD = 1;
                    telemetry.update();
                }
            }

        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        if (Arti_Toli_ToliToli == 1 && KarD == 1) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new šauti(sm1, sm2, 0.5))
                    .strafeTo(new Vector2d(0, -10))
                    .build()
            );
        } else if (Arti_Toli_ToliToli == 1 && KarD == 2) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new šauti(sm1, sm2, 0.5))
                    .strafeTo(new Vector2d(0, 10))
                    .build()
            );
        } else if (Arti_Toli_ToliToli == 2 && KarD == 1) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new šauti(sm1, sm2, 0.5))
                    .strafeTo(new Vector2d(0, -10))
                    .build()
            );
        } else if (Arti_Toli_ToliToli == 2 && KarD == 2) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new šauti(sm1, sm2, 0.5))
                    .strafeTo(new Vector2d(0, 10))
                    .build()
            );
        } else {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new šauti(sm1, sm2, 0.5))
                    .lineToX(10)
                    .build()
            );
        }


    }

    public class šauti implements Action {
        DcMotor sm1, sm2;
        double sp;

        public šauti(DcMotor sm1, DcMotor sm2, double sp) {
            this.sm1 = sm1;
            this.sm2 = sm2;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            resetRuntime();
            int n = 0;
            sm1.setPower(sp);
            sm2.setPower(-sp);
            while (getRuntime() < 2) {
                pm.setPosition(0);
                P1S.setPower(-1);
                P2S.setPower(1);
                if (getRuntime() > 1)
                    pm.setPosition(0.45);
                resetRuntime();
                n++;
                if (n == 3) break;
            } // kas parase sita cikla, tas rizikuoja komandos sansus laimeti, sveikinu NOJAU
            return false;

        }
    }

}