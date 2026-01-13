package org.firstinspires.ftc.teamcode.Mechanizmai;


import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Šaudyklė {
    DcMotorEx sm1,sm2; //, iÅmetimas //0, 1, 2expansion hub
    DcMotor pad, pem; //Paėmimas padavimas

    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800
    double smGalia = 0.85;

    // Start at ~70% power
    private double targetVelocity = MAX_TICKS_PER_SEC * 1;   ///derinsimes
    double x,y; //aprilTag x , y detection
    public Šaudyklė(HardwareMap hwMap) {
        sm1 = hwMap.get(DcMotorEx.class, "svD");  // 0 lizdas expansion hub
        sm2 = hwMap.get(DcMotorEx.class, "svK");  // 1 lizdas expansion hub
        pad = hwMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hwMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub
    }
    public class ugnis implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                sm1.setPower(smGalia);
                sm2.setPower(smGalia);
                sleep(400);
                sm1.setPower(targetVelocity);
                sm2.setPower(targetVelocity);
                pad.setPower(-0.7);
                pem.setPower(-0.5);
                sleep(900);
                sm1.setPower(0);
                sm2.setPower(0);
                pad.setPower(0);
                pem.setPower(0);
                initialized = true;
            }
            return false;
        }
    }
    public  Action ugnis() {
        return new Šaudyklė.ugnis();
    }
    public class atgal1 implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pem.setPower(0.7);
                pad.setPower(0.7);
                sm1.setPower(-0.6);
                sm2.setPower(-0.6);
                initialized = true;
            }
            return false;
        }
    }
    public  Action atgal1() {
        return new Šaudyklė.atgal1();
    }
    public class atgal0 implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pem.setPower(0);
                pad.setPower(-0);
                sm1.setPower(-0);
                sm2.setPower(-0);
                initialized = true;
            }
            return false;
        }
    }
    public  Action atgal0() {
        return new Šaudyklė.atgal0();
    }

}
