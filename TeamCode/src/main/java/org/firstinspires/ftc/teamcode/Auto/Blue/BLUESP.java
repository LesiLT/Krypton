package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Mechanizmai.Kamera;
import org.firstinspires.ftc.teamcode.Mechanizmai.Šaudyklė2;

@Autonomous(name = "BLUESP")
public class BLUESP extends OpMode {
    Šaudyklė2 saudykle = new Šaudyklė2();
    GoBildaPinpointDriver odo;
    DcMotor kP, kG, dP, dG;
    DcMotor pem;
    //Servo kamp;
    Kamera kam;
    Pose2D Pos ;
    double kampas ;
    double P = 0, B = 0, S = 0; ///Primyn /// Į Šoną /// Suktis
    int veiksmas = 0;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pem = hardwareMap.get(DcMotor.class, "pem");
        //kamp = hardwareMap.get(Servo.class, "kamp");

        kP = hardwareMap.get(DcMotor.class, "kP");
        dP = hardwareMap.get(DcMotor.class, "dP");
        kG = hardwareMap.get(DcMotor.class, "kG");
        dG = hardwareMap.get(DcMotor.class, "dG");

        kP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kam = new Kamera(hardwareMap, telemetry);

        saudykle.init(hardwareMap);

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startPos = new Pose2D(DistanceUnit.MM, -8, -9, AngleUnit.RADIANS, 0);
        odo.setPosition(startPos);
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
        telemetry.addData("1Kampas: ", kampas);
    }
    @Override
    public void loop() {
        moveRobot();
        odo.update();
        Pos = odo.getPosition();
        double k = Pos.getX(DistanceUnit.CM);
        double p = Pos.getY(DistanceUnit.CM);
        kampas = Pos.getHeading(AngleUnit.RADIANS);
// atsitraukia, sauna, pasisuka, strafina, issuka paemima, pavaziau i prieki paimti,strafe atgal,pasisuka, issauna, isvaziuoja is linijos
        if(k > -40 && veiksmas == 0){
            P=-0.45;
        }
        else if (k < -40 && veiksmas == 0){
            P = 0;
            saudykle.teleugnis(0.8);
            veiksmas= 1;
            odo.resetPosAndIMU();
        }
        if(p < 35 && veiksmas == 1) {
            B = 0.45;
        }
        else if(p > 45 && veiksmas == 1){
            B = 0;
            veiksmas = 2;
            odo.resetPosAndIMU();
        }
//                if (p < 30 && veiksmas == 2){
//                    B = 0.45;
//                }
//                else if (p > 30 && veiksmas == 2){
//                    B = 0;
//                    pem.setPower(0.5);
//                    veiksmas = 3;
//                    odo.resetPosAndIMU();
//                }
//                if (k < 10 && veiksmas == 3){
//                    P = 0.45;
//                }
//                else if (k > 10 && veiksmas == 3){
//                    P = 0;
//                    pem.setPower(0);
//                    veiksmas = 4;
//                    odo.resetPosAndIMU();
//                }
//                if (p > -25 && veiksmas == 4) {
//                    B = -0.45;
//                }
//                else if(p < -25 && veiksmas == 4){
//                    B = 0;
//                    veiksmas = 5;
//                    odo.resetPosAndIMU();
//                }
//                if(kampas > -0.2 && veiksmas == 5) {
//                    S = 0.45;
//                }
//                else if (kampas < -0.2 && veiksmas == 5){
//                    S = 0;
//                    veiksmas = 6;
//                    odo.resetPosAndIMU();
//                }
        telemetry.addData("Veiksmas", veiksmas );
        telemetry.addData("2Kampas: ", kampas);
    }
}

