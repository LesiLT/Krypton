package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp()
public class MainTeleOp extends LinearOpMode {
    Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas
    Double sp = 0.5; //Greitis
   // CRServo P1S, P2S; // Padavimo servai
   // DcMotor pm, sm; //Paėmimas, išmetimas

    @Override
    public void runOpMode() throws InterruptedException {
        //Važiuoklės varikliai
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub

     /*   //Išmetimas/Paėmimas
        pm = hardwareMap.get(DcMotor.class, "pm");
        sm = hardwareMap.get(DcMotor.class, "sm");

        //Padavimo servai
        P1S = hardwareMap.get(CRServo.class, "P1S");
        P2S = hardwareMap.get(CRServo.class, "P2S");
*/
        waitForStart();
        while (!isStopRequested()) {

            // Važiuoklė
            MecanumDrive drive = new MecanumDrive(kP, dP, kG, dG) ;

           drive.driveFieldCentric(
                   gamepad1.left_stick_x * sp, // strafe/drift
                   gamepad1.left_stick_y * -sp, // priekis
                   gamepad1.right_stick_x * sp, // posūkis
                   0
           );
/*
           //Šaudymas:
            //išmetimas
           if (gamepad1.triangle){
               sm.setPower(1);
           }
           // Padavimas
            if (gamepad1.right_bumper){
                P1S.setPower(0.5);
                P2S.setPower(0.5);
            } else if (!gamepad1.right_bumper) {
                P1S.setPower(0);
                P2S.setPower(0);
            }
            //Paėmimas
            pm.setPower(gamepad1.left_trigger);

            if (0!=0) {


            }*/

        }

    }
}
