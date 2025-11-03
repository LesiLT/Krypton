package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous (name = "Auto tiesiai")
public class AutoPaprastas extends OpMode {

    int kartai = 0;
    DcMotor kP, kG, dP, dG;
    DcMotor sm1,sm2;

    CRServo P1S,P2S;
    Servo pm;
    int x = 0;

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

            resetRuntime();
        pm.setPosition(0);
    }

    @Override
    public void loop() {
        if (getRuntime() > 1 && x == 0) {
            resetRuntime();
            x++;
        }

        while (getRuntime() < 3) {
            kP.setPower(-0.2);
            kG.setPower(-0.2);
            dP.setPower(0.2);
            dG.setPower(0.2);
        }

        kP.setPower(-0);
        kG.setPower(-0);
        dP.setPower(0);
        dG.setPower(0);

}}
