package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mechanizmai.HardwareInit;

@Autonomous(name = "Auto_Clm")
public class AutoClM extends LinearOpMode {
    @Override
    public void runOpMode() {

    Pose2d initialPose = new Pose2d(0, 0, 0);
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    HardwareInit hw;
    waitForStart();
    Actions.runBlocking(
            new SequentialAction(
                    
            )
    );
    }
}
