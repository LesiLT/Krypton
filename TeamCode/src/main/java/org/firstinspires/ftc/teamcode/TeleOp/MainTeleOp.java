package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    //---------------------------------------------------------------------------
    //Paėmimo kampas
    private Servo pKamp;
    //----------------------------------------------------------------------------
    //Paėmimo žnyplė
    private Servo pClaw;
    double pAt = 0;
    double pUz = 0.5;
    //---------------------------------------------------------------------------
    //Paėmimo slide
    private CRServo mid;
    //----------------------------------------------------------------------------
    //pakabinimo letena
    double pakAt = 0.2;
    double pakUz = 0;
    private Servo mainClaw;
    //--------------------------------------------------------------------------
    //Važiuoklė
    double sp = 0.75; //greitis
    private Motor fL, bL, bR, fR; //variklai

    //Liftas
    int pakPos = 1572;
    private Motor upper;
    int slidePosMax = 2490;
    double liftSp = 0.2;

    //Paemimo pozicija
    Servo pPos;
    double paemimoPos = 0.6;
    double pakeltaPos = 0;
    //-------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        //-------------------------------------------------------------------
        pPos = hardwareMap.get(Servo.class, "pPos");
        //--------------------------------------------------------------------
        pClaw = hardwareMap.get(Servo.class, "pClaw");
        //---------------------------------------------------------------------
        pKamp = hardwareMap.get(Servo.class, "pKamp");
        //---------------------------------------------------------------------
        mid = hardwareMap.get(CRServo.class, "mid");
        double midSp = 0.4;
        //---------------------------------------------------------------------
        mainClaw = hardwareMap.get(Servo.class, "mainClaw");
        //---------------------------------------------------------------------
        upper = new Motor(hardwareMap, "upper", Motor.GoBILDA.RPM_312);
        upper.setRunMode(Motor.RunMode.PositionControl);
        upper.setPositionCoefficient(0.01);
        upper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        int upperStartPos = upper.getCurrentPosition();
        //Varikliu aprasas
        fL = new Motor(hardwareMap, "leftFront");
        fR = new Motor(hardwareMap, "rightFront");
        bL = new Motor(hardwareMap, "leftBack");
        bR = new Motor(hardwareMap, "rightBack");

        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //-----------------------------------------------------------------------

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("LiftY:", upper.getCurrentPosition());

            //---------------------------------------------------------------------
            double pakPos = mainClaw.getPosition();
            //---------------------------------------------------------------------
            //Važiuoklė
            MecanumDrive drive = new MecanumDrive(fL, fR, bL, bR);

            drive.driveRobotCentric(
                    gamepad1.left_stick_x * sp,
                    -gamepad1.left_stick_y * sp,
                    -gamepad1.right_stick_x * sp,
                    false
            );
            //-----------------------------------------------------------------------
            //Žnyplių suspaudimas

            if (gamepad1.circle) {
                mainClaw.setPosition(pakAt);
                pClaw.setPosition(pAt);
            } else if (gamepad1.square) {
                mainClaw.setPosition(pakUz);
                pClaw.setPosition(pUz);
            }


            //Liftas
            if (gamepad1.dpad_up) {
                upper.setTargetPosition(upperStartPos - slidePosMax);
                telemetry.update();
                upper.set(sp);
            } else if (gamepad1.dpad_down) {
                upper.setTargetPosition(upperStartPos - 2);
                telemetry.update();
                upper.set(sp);

            } else if (!gamepad1.dpad_up && !gamepad1.dpad_down)
                upper.stopMotor();


            //Paėmimo slide
            if (gamepad1.triangle) {
                mid.setPower(midSp);
            } else if (gamepad1.cross) {
                mid.setPower(-midSp);
            } else if (!gamepad1.triangle && !gamepad1.cross) {
                mid.setPower(0);
            }
            //------------------------------------------------------------------------
            //Paėmimo kampas horizontalus
            if (gamepad1.left_bumper) {
                pKamp.setPosition(0);
            } else if (gamepad1.right_bumper) {
                pKamp.setPosition(0.5);

                //-------------------------------------------------------------------------
                //Paemimo kampas vertikalus
                if (gamepad1.dpad_left) {
                    pPos.setPosition(paemimoPos);
                } else if (gamepad1.dpad_right) {
                    pPos.setPosition(pakeltaPos);
                }
                //----------------------------------------------------------------------------
            }
        }

    }}
