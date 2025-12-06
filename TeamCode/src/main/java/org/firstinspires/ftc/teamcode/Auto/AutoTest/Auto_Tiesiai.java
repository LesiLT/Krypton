package org.firstinspires.ftc.teamcode.Auto.AutoTest;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Disabled
@Autonomous (name = "Auto tiesiai10")
public class Auto_Tiesiai extends LinearOpMode {
DcMotor kP, kG, dP, dG;
    Servo pak0,pak1;
    DcMotor  sm1,sm2;
    DcMotor pad, pem;
@Override
public void runOpMode() throws InterruptedException {
    kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
    kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
    dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
    dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub

    sm1 = hardwareMap.get(DcMotor.class, "svD");
    sm2 = hardwareMap.get(DcMotor.class, "svK");
    pad = hardwareMap.get(DcMotor.class, "pad");
    pem = hardwareMap.get(DcMotor.class,"pem");
    pak0 = hardwareMap.get(Servo.class, "pak0");
    pak1 = hardwareMap.get(Servo.class, "pak1");
    pak1.setDirection(Servo.Direction.REVERSE);
    kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    MecanumDrive drive = new  MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    waitForStart();
    Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
            .lineToX(10)
            .build()
    );
}
}
