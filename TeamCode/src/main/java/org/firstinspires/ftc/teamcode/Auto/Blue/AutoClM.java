package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanizmai.Surinkimas;
import org.firstinspires.ftc.teamcode.Mechanizmai.Šaudyklė;
@Autonomous(name = "Auto_Clm")
public class AutoClM extends LinearOpMode {
    public Servo kamp;
    @Override
    public void runOpMode() throws InterruptedException {
        kamp = hardwareMap.get(Servo.class, "kamp");
        kamp.setPosition(0);

        Surinkimas surinkimas = new Surinkimas(hardwareMap);
        Šaudyklė saudyklė = new Šaudyklė(hardwareMap);
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder pirmas = drive.actionBuilder(initialPose)

                .lineToX(-10)
                .turn(-Math.PI/18)
                .waitSeconds(0);
        TrajectoryActionBuilder antras = pirmas.endTrajectory().fresh()
                .lineToX(-30)
                .turn(Math.PI/3.75)
                .waitSeconds(0);
        TrajectoryActionBuilder trecias = antras.endTrajectory().fresh()
                .strafeTo(new Vector2d(-35, 4))
                .waitSeconds(0);
        TrajectoryActionBuilder ketvirtas = trecias.endTrajectory().fresh()
                .lineToX(-10)
                .waitSeconds(0.1);
        Action pirmasAction = pirmas.build();
        Action antrasAction = antras.build();
        Action treciasAction = trecias.build();
        Action ketvirtasAction = ketvirtas.build();
    waitForStart();
    kamp.setPosition(0.25);
        Actions.runBlocking(
                new SequentialAction(


                new SequentialAction(
                        pirmasAction,
                        antrasAction,
                        treciasAction,
                        ketvirtasAction
                        //saudyklė.autougnis(),
                        //antrasAction
                        ),
                new ParallelAction(
                        //surinkimas.paemimas(),


)));
    }
}
