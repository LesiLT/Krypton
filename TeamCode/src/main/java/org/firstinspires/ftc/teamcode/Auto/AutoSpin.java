package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoSpin extends OpMode {
    double laikas = getRuntime();
    DcMotor kP, kG, dP, dG;
    @Override
    public void init() {
        kP = hardwareMap.get(DcMotor.class, "kP");
        kG = hardwareMap.get(DcMotor.class, "kG");
        dG = hardwareMap.get(DcMotor.class, "dG");
        dP = hardwareMap.get(DcMotor.class, "dP");
        resetRuntime();
    }

    @Override
    public void loop() {
        laikas = getRuntime();
        if (laikas <= 0.5) {
            kP.setPower(0.5);
            kG.setPower(-0.5);
            dG.setPower(0.5);
            dP.setPower(-0.5);
        }
        if (laikas > 1) {
            kP.setPower(-0.5);
            kG.setPower(0.5);
            dG.setPower(-0.5);
            dP.setPower(0.5);
        }
        if (laikas > 2) {
            kP.setPower(0.5);
            kG.setPower(0.5);
            dG.setPower(0.5);
            dP.setPower(0.5);
            resetRuntime();
        }
        if (laikas > 3){
            kP.setPower(-0.5);
            kG.setPower(-0.5);
            dG.setPower(-0.5);
            dP.setPower(-0.5);
        }


    }
}
