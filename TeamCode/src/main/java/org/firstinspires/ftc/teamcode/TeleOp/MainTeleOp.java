package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas
    Double sp = 0.5; //Greitis

    DcMotor pm, sm; //Paėmimas, išmetimas //0, 1 expansion hub


    boolean prev = false;
    boolean motorOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Važiuoklės varikliai
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub

        //Išmetimas/Paėmimas
        pm = hardwareMap.get(DcMotor.class, "pm");
        sm = hardwareMap.get(DcMotor.class, "sm");
        sm.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (!isStopRequested()) {

            // Važiuoklė
            MecanumDrive drive = new MecanumDrive(kP, dP, kG, dG) ;

           drive.driveFieldCentric(
                  - gamepad1.left_stick_x / 2, // strafe/drift
                    gamepad1.left_stick_y / 2, // priekis
                   -gamepad1.right_stick_x / 2, // posūkis
                   0
           );

           //Šaudymas:
//30 laipsniu
            //Paėmimas
            if (gamepad1.left_trigger > 0) {
                pm.setPower(0.75);
            }
            pm.setPower(0);

            //išmetimas
            boolean paspaustas = false;
            paspaustas = gamepad1.triangle;
            if(paspaustas && !prev)
            {
                motorOn = !motorOn;
            }
            prev = paspaustas;
            if(motorOn){
                gamepad1.rumble(500, 500, 600);
                sm.setPower(1);
            }
            else sm.setPower(0);

        }

    }
}
