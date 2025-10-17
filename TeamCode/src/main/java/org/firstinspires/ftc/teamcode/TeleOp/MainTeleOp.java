package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.MotorCommands;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas
    Double sp = 0.5; //Greitis
    int KP=0,KG=0,DP=0,DG=0;
    CRServo P1S,P2S;
    DcMotor  sm1,sm2; //Paėmimas, išmetimas //0, 1, 2expansion hub
    Servo pm;

    boolean prev = false;
    boolean motorOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Važiuoklės varikliai
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub
        /*kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub*/


        //Išmetimas/Paėmimas

        sm1 = hardwareMap.get(DcMotor.class, "sm1");
        sm2 = hardwareMap.get(DcMotor.class, "sm2");
        P1S = hardwareMap.get(CRServo.class, "P1S");
        P2S = hardwareMap.get(CRServo.class, "P2S");
        pm = hardwareMap.get(Servo.class, "pm");



        waitForStart();
        while (!isStopRequested()) {

            // Važiuoklė
            MecanumDrive drive = new MecanumDrive(kP, dP, kG, dG) ;

          if(motorOn == false){ drive.driveFieldCentric(
                  - gamepad1.left_stick_x * 0.75, // strafe/drift
                    gamepad1.left_stick_y * 0.75, // priekis
                   -gamepad1.right_stick_x * 0.75, // posūkis
                   0
           );}

           //Šaudymas:
            //Padavimas
            if (gamepad1.dpad_down) {
                pm.setPosition(0);
            }
            if (gamepad1.dpad_up){
                telemetry.addData("Servo poz", pm.getPosition());
                telemetry.update();
                pm.setPosition(0.45);

            }
            //Paėmimas


            while(gamepad1.square) {
                P1S.setPower(-1);
                P2S.setPower(1);

            }
            if(!gamepad1.square){P1S.setPower(0);
            P2S.setPower(0);}

            //išmetimas
            telemetry.addData("Titas == blogas",gamepad1.right_trigger);
            if(gamepad1.circle)telemetry.update();
            if (gamepad1.left_trigger > 0.2) {
                sm1.setPower(gamepad1.right_trigger);
                sm2.setPower(-gamepad1.right_trigger);
            }

            boolean paspaustas = false;
            paspaustas = gamepad1.triangle;
            if(paspaustas && !prev)
            {
                motorOn = !motorOn;
            }
            prev = paspaustas;
            if(motorOn){
                gamepad1.rumble(500, 500, 600);
                sm1.setPower(0.76);
                sm2.setPower(-0.76);
                drive.driveFieldCentric(
                        KG,KP,DP,DG );


            }
            else {
                sm1.setPower(0);
                sm2.setPower(0);
            }


            //0.48 toliau
            //0.37 arti

        }
    }
}
