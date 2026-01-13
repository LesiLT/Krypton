package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanizmai.HardwareInit;
import org.firstinspires.ftc.teamcode.Mechanizmai.Posūkis_laisniais;
import org.firstinspires.ftc.teamcode.Mechanizmai.Surinkimas;
import org.firstinspires.ftc.teamcode.Mechanizmai.Šaudyklė;

@Autonomous(name = "Auto_Clm")
public class AutoClM extends LinearOpMode {
    public Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas
    @Override
    public void runOpMode() throws InterruptedException {
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub

        kP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        Surinkimas surinkimas = new Surinkimas(hardwareMap);
        Šaudyklė saudyklė = new Šaudyklė(hardwareMap);

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    waitForStart();
    Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(10)
                    .build());
    Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0, 0, 0))
            .turn(Math.toRadians(-135))
                    .build());
    Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .stopAndAdd(saudyklė.ugnis())
                    .build()
            );
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .turn(Math.toRadians(135))
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(10)
                        .build());
    }
}
