package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous
public class auto_test extends LinearOpMode {

    int KarD;
    int NarT;
    int Arti_Toli_ToliToli;
    @Override
    public void runOpMode() throws InterruptedException {
        while(!isStarted())
        {
            if (gamepad1.dpad_up)
            {
                telemetry.addData("Atstumas", "Arti");
                Arti_Toli_ToliToli = 1;
                telemetry.update();
            }
            else if(gamepad1.dpad_left)
            {
                telemetry.addData("Atstumas", "Toli");
                Arti_Toli_ToliToli = 2;
                telemetry.update();
            }
            else if(gamepad1.dpad_down)
            {
                telemetry.addData("Atstumas", "Labai Toli");
                Arti_Toli_ToliToli = 3;
                telemetry.update();
            }

            if(Arti_Toli_ToliToli == 1)
            {
                if(gamepad1.right_bumper)
                {
                    telemetry.addData("Atstumas", "Arti");
                    telemetry.addData("Pusė", "Dešinė");
                    KarD = 1;
                    telemetry.update();
                }
                else if(gamepad1.left_bumper)
                {
                    telemetry.addData("Atstumas", "Arti");
                    telemetry.addData("Pusė", "Kairė");
                    KarD = 2;
                    telemetry.update();
                }
            }
            else if(Arti_Toli_ToliToli == 2)
            {
                if(gamepad1.right_bumper)
                {
                    telemetry.addData("Atstumas", "Toli");
                    telemetry.addData("Pusė", "Dešinė");
                    KarD = 1;
                    telemetry.update();
                }
                else if(gamepad1.left_bumper)
                {
                    telemetry.addData("Atstumas", "Toli");
                    telemetry.addData("Pusė", "Kairė");
                    KarD = 2;
                    telemetry.update();
                }
            }

        }
        waitForStart();
        while(!isStopRequested())telemetry.addData("baka", "kaka");
    }
}
