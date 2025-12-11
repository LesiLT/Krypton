package org.firstinspires.ftc.teamcode.TeleOp.BeKameros;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanizmai.HardwareInit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled
@TeleOp (name = "MainTeleOpBeKameros")
public class MainTeleOp extends LinearOpMode {
    int KP=0,KG=0,DP=0,DG=0;
    CRServo P1S,P2S;
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

    public HardwareInit hw;

    @Override
    public void runOpMode() throws InterruptedException {

        //Hardware inicializacija
        hw = new HardwareInit();
        hw.initializeHardware(hardwareMap);

        waitForStart();
        while (!isStopRequested()) {


            // Važiuoklė
            MecanumDrive drive = new MecanumDrive(hw.kP, hw.dP, hw.kG, hw.dG);

            if (!motorOn) {
                drive.driveRobotCentric(
                        gamepad1.left_stick_x * 0.85, // strafe/drift
                            -gamepad1.left_stick_y * 0.85, // priekis
                            gamepad1.right_stick_x * 0.85 // posūkis
                );
            }

            //Paemimas

            if (gamepad1.right_bumper) {
                hw.pem.setPower(-0.5);
            } else if (!gamepad1.right_bumper) {
                hw.pem.setPower(0);
            }

            //Padavimas
            if (gamepad1.dpad_up) {
                hw.pad.setPower(0.4);
            }
             else if  (gamepad1.triangle){
                hw.pad.setPower(-0.5);
            }   else if (!gamepad1.triangle || gamepad1.dpad_down) {
                 hw.pad.setPower(0);
             }


            //išmetimas
            telemetry.addData("Titas == blogas", gamepad1.right_trigger);
            //double realtime_rpm1 = 6000*sm1.getPower();
            double realtime_rpm2 = 6000 * abs(hw.sm1.getPower());
            telemetry.addData("RPM sm2", realtime_rpm2);
            if (gamepad1.circle) telemetry.update();


            boolean paspaustas = false;
            paspaustas = gamepad1.left_bumper;
            if (paspaustas && !prev) {
                motorOn = !motorOn;
            }
            prev = paspaustas;
            if (motorOn) {
                if (hw.sm2.getPower() >= targetVelocity) {
                    gamepad1.rumble(500, 500, 600);
                }
                hw.sm1.setPower(targetVelocity);
                hw.sm2.setPower(targetVelocity);
                drive.driveRobotCentric(
                        0,
                        0,
                        0
                );

            }
            else{
                hw.sm1.setPower(0);
                hw.sm2.setPower(0);
            }
            if (gamepad1.dpad_left) {
                //pak0.setPosition(0.9); nuline pozicija
                hw.pak0.setPosition(0.4);
                sleep(500);
                hw.pak1.setPosition(0.4);

            }

        }
    }
}