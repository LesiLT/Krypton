package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

@TeleOp
public class MainTeleOp2p_V2 extends LinearOpMode {

    private Servo pV, pClaw, kClaw, pH;
    private CRServo  pSlide;
    private Motor fL, bL, bR, fR;
    int pakPos = 1572;
    private Motor upper;
    int slidePosMax = 2470;
    double liftSp = 0.35;

    double sp = 0.75;//Važiuoklės greitis

    @Override
    public void runOpMode() throws InterruptedException {
        upper = new Motor(hardwareMap, "upper", Motor.GoBILDA.RPM_312);
        upper.setRunMode(Motor.RunMode.PositionControl);
        upper.setPositionCoefficient(0.01);
        upper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        int upperStartPos = upper.getCurrentPosition();

        pSlide = hardwareMap.get(CRServo.class, "mid");

        fL = new Motor(hardwareMap, "leftFront");
        fR = new Motor(hardwareMap, "rightFront");
        bL = new Motor(hardwareMap, "leftBack");
        bR = new Motor(hardwareMap, "rightBack");

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        pV = hardwareMap.get(Servo.class, "pPos");
        pH = hardwareMap.get(Servo.class, "pKamp");
        pClaw = hardwareMap.get(Servo.class, "pClaw");
        kClaw = hardwareMap.get(Servo.class, "mainClaw");



        waitForStart();
        while (!isStopRequested()){

            if (gamepad2.left_trigger > 0.2) {
                upper.set(upperStartPos -  5000);
                upperStartPos = upper.getCurrentPosition();
                upper.set(0.5);
            }

            if (gamepad2.square){
                pClaw.setPosition(0);
                kClaw.setPosition(0.4);
            } if (gamepad2.circle) {
                pClaw.setPosition(0.5);
                kClaw.setPosition(0);
            }

            if (gamepad2.right_stick_y > 0.25) {
                pV.setPosition(0.6);
            }
            if (gamepad2.right_stick_y < -0.25) {
                pV.setPosition(0);
            }

            if (gamepad2.left_bumper) {
                pH.setPosition(0);
            }
            if (gamepad2.right_bumper) {
                pH.setPosition(0.5);
            }

            if (gamepad2.left_stick_y > 0.55) {
                pSlide.setPower(-0.8);
            } else if (gamepad2.left_stick_y < -0.55) {
                pSlide.setPower(0.8);
            } else if (gamepad2.left_stick_y < 0.55 && gamepad2.left_stick_y > -0.85) {
                pSlide.setPower(0.001);
            }

            if (gamepad2.dpad_up) {
                upper.setTargetPosition(upperStartPos - slidePosMax);
                telemetry.update();



                upper.set(liftSp);
            } else if (gamepad2.dpad_down) {
                upper.setTargetPosition(upperStartPos-2);
                telemetry.update();
                upper.set(liftSp);

            } else if (!gamepad2.dpad_up && !gamepad2.dpad_down)
                upper.stopMotor();
            
            MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);

            drive.driveRobotCentric(
                    gamepad1.left_stick_x * sp,
                    -gamepad1.left_stick_y * sp,
                    -gamepad1.right_stick_x * sp,
                    false
            );
        }
    }
}