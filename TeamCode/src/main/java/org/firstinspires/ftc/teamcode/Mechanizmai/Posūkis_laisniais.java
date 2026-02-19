package org.firstinspires.ftc.teamcode.Mechanizmai;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Disabled
public class Posūkis_laisniais {
    double degrees;
    GoBildaPinpointDriver driver;
    GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public Posūkis_laisniais(HardwareMap hardwareMap) {
    }
    public class posukis implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                Math.toRadians(degrees);
                initialized = true;
            }
            return false;
        }
    }
    public  Action posukis() {
        return new Posūkis_laisniais.posukis();
    }
}
