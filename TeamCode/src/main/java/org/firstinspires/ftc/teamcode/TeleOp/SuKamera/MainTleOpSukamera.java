package org.firstinspires.ftc.teamcode.TeleOp.SuKamera;

import static android.os.SystemClock.sleep;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Mechanizmai.Kamera;
import org.firstinspires.ftc.teamcode.Mechanizmai.Surinkimas;
import org.firstinspires.ftc.teamcode.Mechanizmai.Šaudyklė;

@TeleOp (name = "MainTeleOpSuKamera")
public class MainTleOpSukamera extends LinearOpMode {
    Motor kP, kG, dP, dG; //kairÄ— priekis/galas, desinÄ— priekis/galas
    int KP=0,KG=0,DP=0,DG=0;
    Servo pak1, pak0, kamp;
    //--------------------
    boolean prev = false;
    boolean motorOn = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //Važiuoklės varikliai
        kP = new Motor(hardwareMap, "kP", Motor.GoBILDA.RPM_312); // 0 lizdas control hub
        kG = new Motor(hardwareMap, "kG", Motor.GoBILDA.RPM_312); // 1 lizdas control hub
        dP = new Motor(hardwareMap, "dP", Motor.GoBILDA.RPM_312); // 2 lizdas control hub
        dG = new Motor(hardwareMap, "dG", Motor.GoBILDA.RPM_312); // 3 lizdas control hub
        kP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        /// Pakėlimas
        kamp = hardwareMap.get(Servo.class, "kamp");
        pak1 = hardwareMap.get(Servo.class, "pak1");
        pak0 = hardwareMap.get(Servo.class, "pak0");
        pak0.setDirection(Servo.Direction.REVERSE);
        pak1.setDirection(Servo.Direction.REVERSE);



        //Išmetimas/Paėmimas


        Surinkimas surinkimas = new Surinkimas(hardwareMap);
        Šaudyklė saudykle = new Šaudyklė(hardwareMap);
        Kamera kam = new Kamera(hardwareMap, telemetry);
        saudykle.sm1.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
        saudykle.sm2.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);

        kamp.setPosition(0);
        MecanumDrive drive = new MecanumDrive(kP, dP, kG, dG);
        waitForStart();
        while (!isStopRequested()) {

            // VaÅ¾iuoklÄ—
            /// right bumper,dpad up,square,left bumper,dpad left,circle,
            if(!motorOn){ drive.driveRobotCentric(
                    gamepad1.left_stick_x * 0.85, // strafe/drift
                    -gamepad1.left_stick_y * 0.85, // priekis
                    gamepad1.right_stick_x * 0.85

            );
            }
            if(gamepad1.right_bumper){
                saudykle.pem.setPower(-0.6);
            }
            /// Atgal visas
            if (gamepad1.cross){
                saudykle.teleatgal1();
            }
            else if (!gamepad1.cross && !gamepad1.right_bumper){
                saudykle.teleatgal0();
            }

///Padavimas
            if (gamepad1.dpad_up) {
                saudykle.pad.setPower(0.5);
            }
            else if (!gamepad1.dpad_up) {
                saudykle.pad.setPower(0);
            }
            //Taiklumo korekcija
            /// Z=83cm, kampas 0.65 1 kamuoliukas 11.9 voltai
            /// Kamera nemato:
            ///kampas 0.156, 12.68 voltai antras kamuoliukas, abudu per pusę roboto arčiau
            /// kampas 0.2, 0.25, 0.3 - 0
            /// kampas  0.5, 0.55 - antras, 12.63 voltai
            /// kampas 0.6 - 0 per arti
            /// Beveik prie pat:
            /// Kampas 0.1-0.0, du kamuoliai 12.52 voltai
            /// kampas 0.2 atsimušą į sieną įkrenta du, 12.50 voltai
            /// Z = 81.2 cm:
            /// Kampas, 0.3 - 1, 12.44 voltai

            if (gamepad1.left_bumper)
            {
                kam.telemetryAprilTag();
                telemetry.update();

                if (kam.id == 20 || kam.id == 24) {
                        kamp.setPosition(0.4);
                    saudykle.teleugnis();

                        drive.driveRobotCentric(
                                0,
                                0,
                                0
                        );
                }
                else{
                    kamp.setPosition(0.2);
                    saudykle.teleugnis();

                    drive.driveRobotCentric(
                            0,
                            0,
                            0
                    );
                }


            }
            kam.id=0;

            if (gamepad1.dpad_left && gamepad1.circle) {                //pak0.setPosition(0.9); nuline pozicija
                pak0.setPosition(0.4);

                pak1.setPosition(0.4);

            }
        }

    }
}