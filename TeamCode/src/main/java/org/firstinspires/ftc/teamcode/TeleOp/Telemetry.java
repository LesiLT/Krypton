package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Telemetry extends OpMode {
    Motor upper;
    @Override
    public void init() {
        upper = new Motor(hardwareMap, "upper", Motor.GoBILDA.RPM_312);
        upper.setRunMode(Motor.RunMode.PositionControl);
        upper.setPositionCoefficient(0.1);
    }

    @Override
    public void loop() {
        telemetry.addData("Lift y: ", upper.getCurrentPosition());
    }
}
