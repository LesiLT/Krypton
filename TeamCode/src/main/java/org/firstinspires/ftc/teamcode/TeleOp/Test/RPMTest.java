package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RPMTest extends OpMode {

    DcMotor sm;
    double TPR = sm.getCurrentPosition();
    @Override
    public void init() {
        sm = hardwareMap.get(DcMotor.class, "sm1");
       sm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down){

        }
    }
}
/////////////////////////////////////// kampas 35 laipsniu