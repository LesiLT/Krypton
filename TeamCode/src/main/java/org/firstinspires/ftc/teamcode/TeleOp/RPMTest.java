package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RPMTest extends OpMode {
    Motor sm;
    @Override
    public void init() {
        sm = new Motor(hardwareMap, "sm");

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down){
            sm.getCPR();
            sm.set(0.2);
        }
    }
}
