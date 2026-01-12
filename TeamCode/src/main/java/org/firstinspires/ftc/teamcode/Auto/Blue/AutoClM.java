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
import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous(name = "Auto_Clm")
public class AutoClM extends LinearOpMode {
    public Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas
    public DcMotor sm1, sm2; //Paėmimas, išmetimas //0, 1, 2expansion hub
    public DcMotor pad, pem;
    public Servo pak0, pak1; //Pakelimo servas Eh = 0, Eh 1
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

        //Išmetimas/Paėmimas

        sm1 = hardwareMap.get(DcMotor.class, "svD");  // 0 lizdas expansion hub
        sm2 = hardwareMap.get(DcMotor.class, "svK");  // 1 lizdas expansion hub
        pad = hardwareMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub

        /// Pak4limas
        pak0 = hardwareMap.get(Servo.class, "pak0");
        pak1 = hardwareMap.get(Servo.class, "pak1");

        pak0.setDirection(Servo.Direction.REVERSE);
        pak1.setDirection(Servo.Direction.REVERSE);
        Posūkis_laisniais posukis = new Posūkis_laisniais(hardwareMap);
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
