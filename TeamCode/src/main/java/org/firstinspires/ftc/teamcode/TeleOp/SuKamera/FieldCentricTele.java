package org.firstinspires.ftc.teamcode.TeleOp.SuKamera;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Mechanizmai.Kamera;
import org.firstinspires.ftc.teamcode.Mechanizmai.Surinkimas;
import org.firstinspires.ftc.teamcode.Mechanizmai.Šaudyklė;

@TeleOp
public class FieldCentricTele extends LinearOpMode {
    GoBildaPinpointDriver odo;
    DcMotor kP, kG, dP, dG; //kairÄ— priekis/galas, desinÄ— priekis/galas
    int KP=0,KG=0,DP=0,DG=0;
    Servo kamp;
    Servo sviesa;
    double sp;
    //--------------------
    boolean prev = false;
    boolean motorOn = false;
    double value = 0;
    boolean right, rightLast = false;
    boolean left, leftLast = false;

    DistanceSensor distanceSensor;

    public void moveRobot() {

        double pirmyn = -gamepad1.left_stick_y;
        double bausti = -gamepad1.left_stick_x; ///STRAFE
        double posukis = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double kampas = pos.getHeading(AngleUnit.RADIANS);

        double cos = Math.cos((Math.PI / 2) - kampas);
        double sin = Math.sin((Math.PI / 2) - kampas);

        double didBausme = -pirmyn * sin + bausti * cos; ///Global strafe
        double didPirmyn = pirmyn * cos + bausti * sin; /// Global forward

        double kp, dp, kg, dg;

        kp = didPirmyn + didBausme - posukis;
        dp = didPirmyn - didBausme - posukis;
        kg = didPirmyn + didBausme + posukis;
        dg = didPirmyn - didBausme + posukis;

        kP.setPower(-kp);
        dP.setPower(-dp);
        kG.setPower(kg);
        dG.setPower(dg);

        telemetry.addData("X", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y", pos.getY(DistanceUnit.MM));
        telemetry.addData("Kampas: ", kampas);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        odo.setOffsets(-84.0, -168.0,DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startPos = new Pose2D(DistanceUnit.MM, -8, -9, AngleUnit.RADIANS, 0);
        odo.setPosition(startPos);

        //Važiuoklės varikliai

        kP = hardwareMap.get(DcMotor.class, "kP");
        dP = hardwareMap.get(DcMotor.class, "dP");
        kG = hardwareMap.get(DcMotor.class, "kG");
        dG = hardwareMap.get(DcMotor.class, "dG");
        kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// Pakėlimas
        kamp = hardwareMap.get(Servo.class, "kamp");
        sviesa = hardwareMap.get(Servo.class, "sviesa");


        distanceSensor = hardwareMap.get(DistanceSensor.class, "sviesa");

        //Išmetimas/Paėmimas


        Surinkimas surinkimas = new Surinkimas(hardwareMap);
        Šaudyklė saudykle = new Šaudyklė(hardwareMap);
        Kamera kam = new Kamera(hardwareMap, telemetry);
        saudykle.sm1.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
        saudykle.sm2.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);

        kamp.setPosition(0);

        waitForStart();
        while (!isStopRequested()) {

            // VaÅ¾iuoklÄ—
            moveRobot();
            odo.update();
            /// right bumper,dpad up,square,left bumper,dpad left,circle,

            }
            /// ===============Paėmimas===============
            if(gamepad1.right_bumper){
                saudykle.pem.setPower(-0.8);
            }

            /// ==============ATGAL VISAS==============
            if (gamepad1.cross){
                saudykle.teleatgal1();
            }
            else if (!gamepad1.cross && !gamepad1.right_bumper){
                saudykle.teleStop();
            }

            ///==============PADAVIMAS==============
            if (gamepad1.dpad_up) {
                saudykle.pad.setPower(0.5);
           }
            else if (!gamepad1.dpad_up || distanceSensor.getDistance(DistanceUnit.CM) > 2) {
                saudykle.pad.setPower(0);
            }
            if (distanceSensor.getDistance(DistanceUnit.CM) < 8){
                value = 0.5;
            }
            else if (distanceSensor.getDistance(DistanceUnit.CM) > 6){
                value = 0.72;
            }
            ///==============ATSTUMO KOREKCIJA==============

            if (gamepad1.left_bumper)
            {
                kam.telemetryAprilTag();
                telemetry.update();

                if (kam.id == 20 || kam.id == 24) {
                        kamp.setPosition(0.4);
                        sp=1;
                    saudykle.teleugnis(sp);


                }
                else{
                    kamp.setPosition(0.2);
                    sp=0.95;
                    saudykle.teleugnis(sp);

                }
                kamp.setPosition(0);


            }
            kam.id=0;

            right = gamepad1.dpad_right;
            left = gamepad1.dpad_left;
            if(right && !rightLast)
            {
                value += 0.1;
            }
            else if(left && !leftLast)
            {
                value -= 0.1;
            }
            sviesa.setPosition(value);
            leftLast = left;
            rightLast = right;


            //telemetry.clear();
            telemetry.addData("Atstumas (cm)", "%.2f", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Sviesa pozicija", value);
            telemetry.update();
        }

    }

