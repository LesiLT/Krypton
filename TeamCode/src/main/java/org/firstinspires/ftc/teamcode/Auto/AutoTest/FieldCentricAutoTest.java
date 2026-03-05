package org.firstinspires.ftc.teamcode.Auto.AutoTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous(name = "CustomTest")
public class FieldCentricAutoTest extends OpMode {
  //  Šaudyklė2 saudykle = new Šaudyklė2();
    GoBildaPinpointDriver odo;
 //   DistanceSensor distanceSensor;
  //  double value = 0;
    DcMotor kP, kG, dP, dG;
 //   DcMotor pem;
 //   Servo sviesa;
 //   Servo kamp;
    Kamera kam;
    double sp = 1;

    Pose2D Pos ;//= odo0.getPosition();
    double kampas ;// Pos.getHeading(AngleUnit.RADIANS);
    //double k ;//= Pos.getX(DistanceUnit.CM);

    double P = 0, B = 0, S = 0; ///Primyn /// Į Šoną /// Suktis
    int veiksmas = 0;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

       // pem = hardwareMap.get(DcMotor.class, "pem");
       // distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
       // sviesa = hardwareMap.get(Servo.class, "sviesa");
       // kamp = hardwareMap.get(Servo.class, "kamp");

         kP = hardwareMap.get(DcMotor.class, "kP");
         dP = hardwareMap.get(DcMotor.class, "dP");
         kG = hardwareMap.get(DcMotor.class, "kG");
         dG = hardwareMap.get(DcMotor.class, "dG");

        kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kam = new Kamera(hardwareMap, telemetry);

     //   saudykle.init(hardwareMap);

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

        double pirmyn = P;
        double bausti = B; ///STRAFE
        double posukis = S;

        Pose2D pos = odo.getPosition();
        double kampas = pos.getHeading(AngleUnit.RADIANS);

        double cos = Math.cos((Math.PI / 2) + kampas);
        double sin = Math.sin((Math.PI / 2) + kampas);

        double didBausme = -pirmyn * sin + bausti * cos; ///Global strafe
        double didPirmyn = pirmyn * cos + bausti * sin; /// Global forward

        double kp, dp, kg, dg;

        kp = didPirmyn + didBausme - posukis;
        dp = didPirmyn - didBausme - posukis;
        kg = didPirmyn + didBausme + posukis;
        dg = didPirmyn - didBausme + posukis;

        kP.setPower(-kp * 0.4);
        dP.setPower(-dp * 0.4);
        kG.setPower(kg * 0.4);
        dG.setPower(dg * 0.4);

        telemetry.addData("X", pos.getX(DistanceUnit.CM));
        telemetry.addData("Y", pos.getY(DistanceUnit.CM));
        telemetry.addData("Kampas: ", kampas);
//        double posicia = 60;
//        if(posicia == pos.getX(DistanceUnit.CM) || posicia > pos.getY(DistanceUnit.CM)){
//            odo.resetPosAndIMU();
//        }

    }

        @Override
        public void loop() {
                moveRobot();
                odo.update();
                Pos = odo.getPosition();
                double k = Pos.getX(DistanceUnit.CM);
                double p = Pos.getY(DistanceUnit.CM);
                kampas = Pos.getHeading(AngleUnit.RADIANS);

                if(k < 30 && veiksmas == 0){
                    P=0.4;
                }
                else if (k > 30 && veiksmas == 0){
                    P=0;
                    veiksmas= 1;
                    odo.resetPosAndIMU();                }
                if(k > -30 && veiksmas == 1){
                    P=-0.4;
                }
                else if (k < -30 && veiksmas == 1){
                    P=0;
                    veiksmas= 2;
                    odo.resetPosAndIMU();            }
                if(p > -30 && veiksmas == 2){
                    B=-0.6;
                }
                else if (p < -30 && veiksmas == 2){
                    B=0;
                    veiksmas= 3;
                    odo.resetPosAndIMU();            }
                if(p < 30 && veiksmas == 3){
                    B=0.6;
                }
                else if (p > 30 && veiksmas == 3){
                    B=0;
                    veiksmas= 4;
                    odo.resetPosAndIMU();            }
                if(kampas < 2 && veiksmas == 4){
                    S=-0.4;
                }
                else if (kampas > 0.4 && veiksmas == 4){
                    S=0;
                    veiksmas= 5;
                    odo.resetPosAndIMU();            }
                if(kampas > -0.4 && veiksmas == 5){
                    S=0.4;
                }
                else if (kampas < -2 && veiksmas == 5){
                    S=0;
                    veiksmas= 0;
                    odo.resetPosAndIMU();            }

        }

}
