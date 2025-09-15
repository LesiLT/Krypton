package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous
public class AutoV2 extends LinearOpMode {
    double pakAt = 0.2;
    double pakUz = 0;
    private Motor upper;
    private Servo mainClaw;
    private CRServo mid;
    private RevColorSensorV3 at;
    int maxUp = 2470;
    int pPos = 1289;
    private DcMotor fL, bL, bR, fR;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        mainClaw = hardwareMap.get(Servo.class, "mainClaw");

        at = hardwareMap.get(RevColorSensorV3.class, "at");

        mid = hardwareMap.get(CRServo.class, "mid");

        upper = new Motor(hardwareMap, "upper", Motor.GoBILDA.RPM_312);
        upper.setRunMode(Motor.RunMode.PositionControl);
        upper.setPositionCoefficient(0.2);
        upper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        fL = hardwareMap.get(DcMotor.class, "leftFront");
        fR = hardwareMap.get(DcMotor.class,  "rightFront");
        bL = hardwareMap.get(DcMotor.class, "leftBack");
        bR = hardwareMap.get(DcMotor.class, "rightBack");

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int USP = upper.getCurrentPosition();

        Actions.runBlocking(new ServoAction(mainClaw, pakUz));

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(25),
                new AngularVelConstraint(Math.PI/2)
        ));

        waitForStart();
//        if (!isStopRequested()) {
//            mid.setPower(-0.08);
//        }
            Actions.runBlocking(drive.actionBuilder(beginPose)
                    .stopAndAdd(new upperAction(upper, USP - maxUp))
                    .lineToX(24)
                    .stopAndAdd(new privažavimas(fL, bL, fR, bR, at))
                    .stopAndAdd(new upperAction(upper, USP - pPos))
                    .stopAndAdd(new ServoAction(mainClaw, pakAt))
                    .lineToX(17)
                 //   .stopAndAdd(new upperAction(upper, USP - 5))
                    .strafeTo(new Vector2d(17, -42))
                    .build()
            );

            Actions.runBlocking(drive.actionBuilder(new Pose2d(17, -42, 0))
                    .lineToX(40)
                            .turn(3.141593)
                    .strafeTo(new Vector2d(40, -50))
                    .build()
            );
            Actions.runBlocking(drive.actionBuilder(new Pose2d(40, -50, 3.141593))
                    .lineToX(-8)
                    .lineToX(40)
                    .build()
            );
            Actions.runBlocking(drive.actionBuilder(new Pose2d(40, -50, 3.141593))
                    .strafeTo(new Vector2d(40, -60))
                    .build()
            );
            Actions.runBlocking(drive.actionBuilder(new Pose2d(40, -60, 3.141593))
                    .lineToX(-7)
                    .lineToX(40)
                    .build()
            );
            Actions.runBlocking(drive.actionBuilder(new Pose2d(40, -60, 3.141593))
                .strafeTo(new Vector2d(38, -67))
                .build()
          );
        Actions.runBlocking(drive.actionBuilder(new Pose2d(40, -67, 3.141593))
                .lineToX(-9)
                .build()
        );
            if (isStopRequested()) return;


    }
    //----------------------------------------------------------------------

    public class privažavimas implements Action{
        DcMotor fL, bL, fR, bR;
        RevColorSensorV3 at;


        public privažavimas(DcMotor fL, DcMotor bL, DcMotor fR, DcMotor bR, RevColorSensorV3 at) {
            this.fL = fL;
            this.fR = fR;
            this.bL = bL;
            this.bR = bR;
            this.at = at;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            while (at.getDistance(DistanceUnit.CM) > 8) {
                fL.setPower(0.2);
                fR.setPower(0.2);
                bR.setPower(0.2);
                bL.setPower(0.2);

            }
            if (at.getDistance(DistanceUnit.CM) < 3) {
                fL.setPower(0);
                fR.setPower(0);
                bR.setPower(0);
                bL.setPower(0);
            }
            return false;
        }
    }
    public class ServoAction implements Action {
        Servo servo;
        double postion;

        public ServoAction(Servo s, double p){
            this.servo = s;
            this.postion = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(postion);
            return servo.getPosition() !=postion;
        }
    }
    //-----------------------------------------------------------------------
    public class upperAction implements Action{
        Motor upper;
        int positon;

        public upperAction(Motor u, int p){
            this.upper = u;
            this.positon = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            upper.setTargetPosition(positon);
            while (!upper.atTargetPosition()){
                upper.set(0.2);
            }
            upper.stopMotor();
            return false;
        }
    }
    }
