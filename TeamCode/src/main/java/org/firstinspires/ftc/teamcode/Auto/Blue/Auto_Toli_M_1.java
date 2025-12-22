package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanizmai.HardwareInit_Metodai;

//Cia aprasysiu skirtuma tarp parralel ir sequential actions ir kaip juos aprasyti:
//Sequential:
//Actions.runBlocking(
//    drive.actionBuilder(new Pose2d(0, 0, 0))
//        .lineToX(-27)
//        .stopAndAdd(new šauti(sm1, sm2, pad, pem, 0.9))
//        .lineToX(-33)
//        .stopAndAdd(new suktis(kP, kG, dG, dP, 1))
//        .build()
//);
//
//Parallel:
//
//Actions.runBlocking(
//    drive.actionBuilder(new Pose2d(0, 0, 0))
//        .parallel(  // start a parallel block
//            new ParallelAction(
//                new šauti(sm1, sm2, pad, pem, 0.9), // custom action 1
//                drive.actionBuilder(new Pose2d(0, 0, 0))  // judejimas kartu su sovimu
//                    .lineToX(-27)
//                    .build()
//            )
//        )
//        .stopAndAdd(new suktis(kP, kG, dG, dP, 1)) // Sequential action po paraleliniu
//        .build()
//);
//Pavyzdys imtas is AUTO2MO
@Disabled
@Autonomous
public class Auto_Toli_M_1 extends LinearOpMode {

    public HardwareInit_Metodai hw;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new HardwareInit_Metodai(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

    }
}
