package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Auto")
public class AutoPaprastas extends OpMode {

    int kartai = 0;
    DcMotor kP, kG, dP, dG;
    DcMotor sm1,sm2;

    CRServo P1S,P2S;
    Servo pm;

    @Override
    public void init() {
        kP = hardwareMap.get(DcMotor.class, "kP"); // 0 lizdas control hub
        kG = hardwareMap.get(DcMotor.class, "kG");// 1 lizdas control hub
        dP = hardwareMap.get(DcMotor.class, "dP"); // 2 lizdas control hub
        dG = hardwareMap.get(DcMotor.class, "dG"); // 3 lizdas control hub

        sm1 = hardwareMap.get(DcMotor.class, "sm1");
        sm2 = hardwareMap.get(DcMotor.class, "sm2");
        P1S = hardwareMap.get(CRServo.class, "P1S");
        P2S = hardwareMap.get(CRServo.class, "P2S");
        pm = hardwareMap.get(Servo.class, "pm");

        kartai =0;

        telemetry.addData("kartai ", kartai);

        pm.setPosition(0);
    }

    @Override
    public void loop() {
    telemetry.addData("T: ", getRuntime());

        resetRuntime();
telemetry.addData("Kartai", kartai);
        if (kartai < 3 ) {
            telemetry.addData("Šauna", 0);
            P1S.setPower(-1);
            P2S.setPower(1);
            sm1.setPower(0.76);
            sm2.setPower(-0.76);
            pm.setPosition(0.45);
            telemetry.addData("servo", pm.getPosition());
                if (pm.getPosition() == 0.45 && getRuntime() < 1200){
                    pm.setPosition(0);
                    kartai++;
                }
            }



        P1S.setPower(0);
        P2S.setPower(0);
        sm1.setPower(0);
        sm2.setPower(0);

        resetRuntime();
//        while (kartai >= 3 && getRuntime() < 400){
//            kP.setPower(0.7);
//            kG.setPower(-0.7);
//            dP.setPower(-0.7);
//            dG.setPower(0.7);
//        }
        kP.setPower(0);
        kG.setPower(0);
        dP.setPower(0);
        dG.setPower(0);

    }
}
