package org.firstinspires.ftc.teamcode.Mechanizmai;

import com.acmerobotics.roadrunner.Action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Surinkimas {
    DcMotor pem;
    public Surinkimas(HardwareMap hardwareMap) {
        pem = hardwareMap.get(DcMotor.class, "pem");

    }
    public class paemimas implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pem.setPower(0.6);
                //timer.reset();
                initialized = true;
            }
            return false;
        }
    }
    public  Action paemimas() {
        return new Surinkimas.paemimas();
    }
    public class atgal implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pem.setPower(0.5);
                initialized = true;
            }
            return false;
        }
    }
    public  Action atgal() {
        return new Surinkimas.atgal();
    }
    public class stop implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pem.setPower(0);
                initialized = true;
            }
            return false;
        }
    }
    public  Action stop() {
        return new Surinkimas.stop();
    }
}
