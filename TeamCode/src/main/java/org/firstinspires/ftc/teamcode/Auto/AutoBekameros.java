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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous (name = "AutoBeKamerosTest")
public class AutoBekameros extends LinearOpMode {

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

        //Išmetimas/Paėmimas
        sm1 = hardwareMap.get(DcMotorEx.class, "svD");  // 0 lizdas expansion hub
        sm2 = hardwareMap.get(DcMotorEx.class, "svK");  // 1 lizdas expansion hub
        pad = hardwareMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub
        //Kr1pton?5


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(-10)
                            .stopAndAdd(new šauti(sm1, sm2, pad, pem, 0.6))


            .build());

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
            sleep(3000);
                sm1.setPower(sp);
                sm2.setPower(-sp);
                pad.setPower(0.6);
                pem.setPower(0.6);
            sleep(6000);

            return false;

        }
    }
}