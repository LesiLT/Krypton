package org.firstinspires.ftc.teamcode.TeleOp.SuKamera;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanizmai.HardwareInit_Metodai;

@TeleOp (name = "MainTeleOpSuKamera")
public class MainTleOpSukamera extends LinearOpMode {

    private HardwareInit_Metodai hw;
    DcMotor pem;
    int KP=0,KG=0,DP=0,DG=0;

    //--------------------
    boolean prev = false;
    boolean motorOn = false;

    boolean lastRight = false;
    boolean lastLeft = false;

    double level = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub

        hw = new HardwareInit_Metodai(hardwareMap, telemetry);


        hw.sm1.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
        hw.sm2.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);

        // VaÅ¾iuoklÄ—
        MecanumDrive drive = new MecanumDrive(hw.kP, hw.dP, hw.kG, hw.dG);
        waitForStart();
        while (!isStopRequested()) {



            /// right bumper,dpad up,square,left bumper,dpad left,circle,
            if(!motorOn){ drive.driveRobotCentric(
                    gamepad1.left_stick_x * 0.85, // strafe/drift
                    -gamepad1.left_stick_y * 0.85, // priekis
                    gamepad1.right_stick_x * 0.85

            );
            }///Paemimas
            if (gamepad1.right_bumper) {
                hw.atgal2();
            }
            else if(gamepad1.triangle) pem.setPower(0.5);
            else if (!gamepad1.right_bumper && !gamepad1.triangle) {
                pem.setPower(0);
            }
            /// Atgal visas
            if (gamepad1.cross){
                hw.atgal1();
            }
            else if (!gamepad1.cross){
                hw.atgal0();
            }


            if (gamepad1.dpad_up) {
                hw.pad.setPower(0.5);  ///Padavimas
            } else if (!gamepad1.dpad_up) {
                hw.pad.setPower(0);
            }



            if (gamepad1.square) {
                if (hw.sm2.getVelocity() >= hw.targetVelocity) {
                    gamepad1.rumble(500, 500, 600);
                }
               hw.ugnis();
                drive.driveRobotCentric(
                        0,
                        0,
                        0
                );

            }



            //Taiklumo korekcija
            while (gamepad1.left_bumper)
            {
                double sp = 0.2; //Greitis
                hw.telemetryAprilTag();
                telemetry.update();
                if (hw.id == 20 || hw.id == 24) {
                    if (hw.x >= -18 && hw.x <= 18) {
                        hw.ugnis();
                        drive.driveRobotCentric(
                                0,
                                0,
                                0
                        );
                    } else if (hw.x < - 5 || hw.x > 5) {
                        break;

                    }
                }


            }
            hw.id=0;


            /*if(gamepad1.dpad_right && !lastRight)
            {
                level = level + 0.05;
            }

            if(gamepad1.dpad_left && !lastLeft)
            {
                level = level - 0.05;
            }

            hw.kamp.setPosition(level);

            lastRight = gamepad1.dpad_right;
            lastLeft = gamepad1.dpad_left;*/

            if (gamepad1.dpad_left && gamepad1.circle) {
                //hw.pak0.setPosition(0.9); nuline pozicija
                hw.pak0.setPosition(0.4);

                hw.pak1.setPosition(0.4);

            }




        }

    }
}
