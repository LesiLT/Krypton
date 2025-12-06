package org.firstinspires.ftc.teamcode.Auto.AutoTest;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
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
        //Kr1pton?5
        while (!isStarted()) {
            pm.setPosition(0);
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
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .strafeTo(new Vector2d(0, -10))
                    .build()
            );
        } else if (Arti_Toli_ToliToli == 1 && KarD == 2) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .strafeTo(new Vector2d(0, 10))
                    .build()
            );
        } else if (Arti_Toli_ToliToli == 2 && KarD == 1) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .strafeTo(new Vector2d(0, -10))
                    .build()
            );
        } else if (Arti_Toli_ToliToli == 2 && KarD == 2) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .strafeTo(new Vector2d(0, 10))
                    .build()
            );
        } else {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new keliauti(P1S,P2S,pm))
                    .stopAndAdd(new šauti(sm1, sm2,pm, 0.5))
                    .stopAndAdd(new atgal(kP,kG,dP,dG))
                            .stopAndAdd(new pirmyn(kP,kG,dP,dG))

            .build());
                    /*kP.setPower(0.5);
            kG.setPower(0.5);
            dP.setPower(0.5);
            dG.setPower(0.5);
                    sleep(1500);
            kP.setPower(0);
            kG.setPower(0);
            dP.setPower(0);
            dG.setPower(0);*/

        }


    }

    public class keliauti implements Action {
        CRServo P1S, P2S;
        Servo pm;
        public keliauti(CRServo P1S, CRServo P2S, Servo pm) {
            this.P1S = P1S;
            this.P2S = P2S;
            this.pm = pm;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pm.setPosition(0);
            resetRuntime();
            getRuntime();

            while (getRuntime() < 2) {
                P1S.setPower(-1);
                P2S.setPower(1);
            }
            P1S.setPower(0);
            P2S.setPower(0);
            resetRuntime();
            return false;
        }



    }
    public class šauti implements Action {
        DcMotor sm1, sm2;
        Servo pm;
        double sp;

        public šauti(DcMotor sm1, DcMotor sm2,Servo pm, double sp) {
            this.sm1 = sm1;
            this.sm2 = sm2;
            this.pm = pm;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                getRuntime();
                sm1.setPower(0.6);
                sm2.setPower(-0.6);
                sleep(1000);
                pm.setPosition(0.45);
                resetRuntime();
                sleep(1000);
                sm1.setPower(0);
                sm2.setPower(0);





            return false;

        }
    }
    public class pirmyn implements Action {
        DcMotor kP, kG, dP, dG;

        public pirmyn(DcMotor kP, DcMotor kG,DcMotor dP, DcMotor dG) {
            this.kP = kP;
            this.kG =kG;
            this.dP = dP;
            this.dG =dG;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            getRuntime();
            while (getRuntime() <0.5) {
                kP.setPower(0.5);
                kG.setPower(-0.5);
                dP.setPower(0.5);
                dG.setPower(-0.5);

            }
            resetRuntime();



            return false;

        }}    public class atgal implements Action {
        DcMotor kP, kG, dP, dG;

        public atgal(DcMotor kP, DcMotor kG,DcMotor dP, DcMotor dG) {
            this.kP = kP;
            this.kG =kG;
            this.dP = dP;
            this.dG =dG;


        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            resetRuntime();
            while (getRuntime() <1.5) {
                kP.setPower(0.5);
                kG.setPower(0.5);
                dP.setPower(-0.5);
                dG.setPower(-0.5);

            }
            resetRuntime();



            return false;

        }}

}