package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.MotorCommands;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Kamera.AprilLibrary;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name = "Main be korekcijos")
public class MainTeleOp extends LinearOpMode {
    Motor kP, kG, dP, dG; //kairė priekis/galas, desinė priekis/galas

    int KP=0,KG=0,DP=0,DG=0;
    CRServo P1S,P2S;
    DcMotor  sm1,sm2; //Paėmimas, išmetimas //0, 1, 2expansion hub
    DcMotor pad, pem;
    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    // Start at ~70% power
    private double targetVelocity = MAX_TICKS_PER_SEC * 0.7;   ///derinsimes
    boolean prev = false;
    boolean motorOn = false;
    //AprilTag skirti dalykai
    double x; //aprilTag x detection
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    double sp = 0.5;

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


        sm1 = hardwareMap.get(DcMotor.class, "svD");  // 0 lizdas expansion hub
        sm2 = hardwareMap.get(DcMotor.class, "svK");  // 1 lizdas expansion hub
        pad = hardwareMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hardwareMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub


        waitForStart();
        while (!isStopRequested()) {


            // Važiuoklė
            MecanumDrive drive = new MecanumDrive(kP, dP, kG, dG);

            if (!motorOn) {
                drive.driveFieldCentric(
                        gamepad1.left_stick_x * 0.85, // strafe/drift
                        -gamepad1.left_stick_y * 0.85, // priekis
                        gamepad1.right_stick_x * 0.85, // posūkis
                        0
                );
            }

            //Paemimas

            if (gamepad1.right_bumper) {
                pem.setPower(-0.5);
            } else if (!gamepad1.right_bumper) {
                pem.setPower(0);
            }

            //Padavimas
            if (gamepad1.dpad_up) {
                pad.setPower(0.4);
            }

            if (gamepad1.dpad_down) {
                pad.setPower(0);
            }


            //išmetimas
            telemetry.addData("Titas == blogas", gamepad1.right_trigger);
            //double realtime_rpm1 = 6000*sm1.getPower();
            double realtime_rpm2 = 6000 * abs(sm1.getPower());
            telemetry.addData("RPM sm2", realtime_rpm2);
            if (gamepad1.circle) telemetry.update();


            boolean paspaustas = false;
            paspaustas = gamepad1.left_bumper;
            if (paspaustas && !prev) {
                motorOn = !motorOn;
            }
            prev = paspaustas;
            if (motorOn) {
                if (sm2.getPower() >= targetVelocity) {
                    gamepad1.rumble(500, 500, 600);
                }
                sm1.setPower(-targetVelocity);
                sm2.setPower(targetVelocity);
                drive.driveFieldCentric(
                        KG, KP, DP, DG);


            }
            else{
                sm1.setPower(0);
                sm2.setPower(0);
            }
        }}}