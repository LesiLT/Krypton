package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled

@TeleOp
public class Telemetrija extends OpMode {
    Servo pak0, pak1;
    @Override
    public void init() {
        pak0 = hardwareMap.get(Servo.class, "pak0");
        pak1 = hardwareMap.get(Servo.class, "pak1");
    }

    @Override
    public void loop() {

        telemetry.addData("Kairys X: ", gamepad1.left_stick_x);
        telemetry.addData("Kairys Y: ",  gamepad1.left_stick_y);
        telemetry.addData("Dešinus X:", gamepad1.right_stick_x);
        telemetry.addData("Desinys Y: ", gamepad1.right_stick_y);
        telemetry.addData("Priekinis: ", pak0.getPosition());
        telemetry.addData("Galinis ", pak1.getPosition());
        telemetry.update();
        pak0.setPosition(gamepad1.left_trigger);
        pak1.setPosition(gamepad1.right_trigger);
    }
}
