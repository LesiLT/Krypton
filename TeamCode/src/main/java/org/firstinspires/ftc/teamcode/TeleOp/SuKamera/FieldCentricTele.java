package org.firstinspires.ftc.teamcode.TeleOp.SuKamera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Mechanizmai.Kamera;
import org.firstinspires.ftc.teamcode.Mechanizmai.Šaudyklė2;


@TeleOp
public class FieldCentricTele extends OpMode {
    Šaudyklė2 saudykle = new Šaudyklė2();
    GoBildaPinpointDriver odo;
    DistanceSensor distanceSensor;
    double value = 0;
    double x;
    DcMotor kP, kG, dP, dG;
    DcMotor pem;
    Servo sviesa;
    //Servo kamp;
    Kamera kam;
    boolean right, rightLast = false;
    boolean left, leftLast = false;
    double sp = 1;
    
    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pem = hardwareMap.get(DcMotor.class, "pem");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        sviesa = hardwareMap.get(Servo.class, "sviesa");
        //kamp = hardwareMap.get(Servo.class, "kamp");

         kP = hardwareMap.get(DcMotor.class, "kP");
         dP = hardwareMap.get(DcMotor.class, "dP");
         kG = hardwareMap.get(DcMotor.class, "kG");
         dG = hardwareMap.get(DcMotor.class, "dG");

        kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //kam = new Kamera(hardwareMap, telemetry);

        saudykle.init(hardwareMap);

        odo.setOffsets(-40.0, -151.0, DistanceUnit.MM);
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

        double cos = Math.cos((Math.PI / 2) + kampas);// (-) buvo
        double sin = Math.sin((Math.PI / 2) + kampas);

        double didBausme = -pirmyn * sin + bausti * cos; ///Global strafe
        double didPirmyn = pirmyn * cos + bausti * sin; /// Global forward

        double kp, dp, kg, dg;

        kp = didPirmyn + didBausme - posukis;
        dp = didPirmyn - didBausme - posukis;
        kg = didPirmyn + didBausme + posukis;
        dg = didPirmyn - didBausme + posukis;

        kP.setPower(-kp*0.8);
        dP.setPower(-dp*0.8);
        kG.setPower(kg*0.8);
        dG.setPower(dg*0.8 );

        telemetry.addData("X", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y", pos.getY(DistanceUnit.MM));
        telemetry.addData("Kampas: ", kampas);

    }

        @Override
        public void loop() {
                moveRobot();
                odo.update();
                if (gamepad1.options){
                    odo.resetPosAndIMU();
                }

            /// ===============Paėmimas===============
            if(gamepad1.right_bumper){
                saudykle.pem.setPower(0.65);
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
//            x = kam.x;
//            if (x == 30){
//                value =0.72;
//
//            }
//            else value =0.5;
            if (gamepad1.left_bumper)
            {
//                kam.telemetryAprilTag();
//                telemetry.update();

//                if (kam.id == 20 || kam.id == 24) {
//                    //kamp.setPosition(0.4);
//                    sp=0.9;
//                    saudykle.teleugnis(sp);
//
//                }

                    //kamp.setPosition(0.2);
                    sp=0.9;
                    saudykle.teleugnis(sp);


                //kamp.setPosition(0);


            }
            //kam.id=0;
//            right = gamepad1.dpad_right;
//            left = gamepad1.dpad_left;
//            if(right && !rightLast)
//            {
//                value += 0.1;
//            }
//            else if(left && !leftLast)
//            {
//                value -= 0.1;
//            }
            //value = 0.72;
            sviesa.setPosition(value);
            //leftLast = left;
            //rightLast = right;


            //telemetry.clear();
            //telemetry.addData("Atstumas (cm)", "%.2f", kam.z);
            //telemetry.addData(" ",kam.id);
            telemetry.addData("Sviesa pozicija", value);
            telemetry.update();

        }

}
