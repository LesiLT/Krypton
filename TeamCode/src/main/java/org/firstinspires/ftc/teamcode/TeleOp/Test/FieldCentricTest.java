package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp
public class FieldCentricTest extends OpMode {

    GoBildaPinpointDriver odo;

    DcMotor kP, kG, dP, dG;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

         kP = hardwareMap.get(DcMotor.class, "kP");
         dP = hardwareMap.get(DcMotor.class, "dP");
         kG = hardwareMap.get(DcMotor.class, "kG");
         dG = hardwareMap.get(DcMotor.class, "dG");

        kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        kP.setDirection(DcMotorSimple.Direction.REVERSE);
//        dP.setDirection(DcMotorSimple.Direction.REVERSE);
//        dG.setDirection(DcMotorSimple.Direction.REVERSE);
//        kG.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startPos = new Pose2D(DistanceUnit.MM, -8, -9, AngleUnit.RADIANS, 0);
        odo.setPosition(startPos);

//        telemetry.addData("X", odo.getXOffset());
//        telemetry.addData("Y", odo.getYOffset());
//        telemetry.addData("Versija: ", odo.getDeviceVersion());
//        telemetry.addData("Scalar", odo.getYawScalar());

    }

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
        public void loop() {
            moveRobot();

            odo.update();
        }

}
