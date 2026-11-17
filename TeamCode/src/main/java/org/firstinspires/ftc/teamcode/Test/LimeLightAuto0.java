package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name="LimeLightAuto_Test")
public class LimeLightAuto0 extends LinearOpMode {

    //Limelight'o aprasymas
    Limelight3A limelight;
    //Motoru aprasymas
    DcMotor fL, bL, bR, fR;

    //Roboto pozicija
    double robotY = 20.0;
    double robotX = 20.0;
    double robotHeading = 0.0;

    //Roboto maksimalios koordinates, kad neperzengtu i priesu teritorija
    final double MIN_X = 0.0;
    final double MAX_X = 20.0;
    final double MIN_Y = 0.0;
    final double MAX_Y = 20.0;

    final double AUTO_TIME_LIMIT = 29.0;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        fL = hardwareMap.get(DcMotor.class, "leftFront");
        fR = hardwareMap.get(DcMotor.class,  "rightFront");
        bL = hardwareMap.get(DcMotor.class, "leftBack");
        bR = hardwareMap.get(DcMotor.class, "rightBack");

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(robotX, robotY, robotHeading);

        // 0 yra geltoniem kamuoliam
        limelight.pipelineSwitch(0);

        // startas rezultatams
        limelight.start();

        telemetry.addLine("Limelight ijungtas");
        telemetry.update();

        waitForStart();

        while(opModeIsActive() && getRuntime() < AUTO_TIME_LIMIT)
        {
            LLResult result = limelight.getLatestResult();

            if(result != null && result.isValid())
            {
                List<LLResultTypes.ColorResult> colors = result.getColorResults();
                if(!colors.isEmpty())
                {
                    double targetX = colors.get(0).getTargetXDegrees();
                    lookAtYellowBall(targetX, fL, fR, bL, bR);
                }

            }
        }

    }

    public void lookAtYellowBall(
            double targetX,
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight) {

        //  Galia posukio
        double kP = 0.015;

        // max sukimosi galia
        double maxPower = 0.3;

        // rotacija
        double turn = targetX * kP;
        turn = Math.max(-maxPower, Math.min(maxPower, turn));

        // paklaidu ignoravimas
        if (Math.abs(targetX) < 1.5) {
            turn = 0;
        }
        frontLeft.setPower(turn);
        backLeft.setPower(turn);

        frontRight.setPower(-turn);
        backRight.setPower(-turn);
    }
}
