package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Telemetry extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {

        telemetry.addData("Kairys X: ", gamepad1.left_stick_x);
        telemetry.addData("Kairys Y: ",  gamepad1.left_stick_y);
        telemetry.addData("Dešinus X:", gamepad1.right_stick_x);
        telemetry.addData("Desinys Y: ", gamepad1.right_stick_y);
    }
}
