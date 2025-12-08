package org.firstinspires.ftc.teamcode.Auto.AutoTest;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Auto10 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        while (!isStopRequested()){
        if (gamepad1.dpad_up) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(10)
                    .build()
            );
            /// 30.5 cm = 10
        }
        if (gamepad1.dpad_left) {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .strafeTo(new Vector2d(0,10))
                    .build()
            ); /// 71.5 = 10
        }
        if (gamepad1.dpad_down){
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .turn(1)
                    .build()
            );
        }
    }}
}
