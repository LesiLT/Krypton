package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
        kamp.setPosition(0.15);

        Surinkimas surinkimas = new Surinkimas(hardwareMap);
        Šaudyklė saudyklė = new Šaudyklė(hardwareMap);

        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder PRisovimas = drive.actionBuilder(initialPose)
                        .lineToX(10)
                                .turn(-Math.PI);
        TrajectoryActionBuilder Atsitraukimas = PRisovimas.endTrajectory().fresh()
                .lineToX(-10)
                .strafeTo(new Vector2d(0, 5));
        Action PRisovimasAction = PRisovimas.build();
        Action AtsitraukimasAction = Atsitraukimas.build();




    waitForStart();
    kamp.setPosition(0.15);

        Actions.runBlocking(
                        new ParallelAction(
                                PRisovimasAction,
                                saudyklė.autougnis(),
                                AtsitraukimasAction

                                        //saudyklė.ugnis(),
                                        //AtsitraukimasAction


                        ));
    }
}
