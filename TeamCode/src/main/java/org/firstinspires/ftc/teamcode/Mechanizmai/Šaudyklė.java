package org.firstinspires.ftc.teamcode.Mechanizmai;


import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Šaudyklė {
    public DcMotorEx sm1,sm2; //, iÅmetimas //0, 1, 2expansion hub
    public DcMotor pad, pem; //Paėmimas padavimas

    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800
    double smGalia = 0.85;
    double sp;

    // Start at ~70% power
    private double targetVelocity = MAX_TICKS_PER_SEC * 1;   ///derinsimes
    double x,y; //aprilTag x , y detection
    public Šaudyklė(HardwareMap hwMap) {
        sm1 = hwMap.get(DcMotorEx.class, "svD");  // 0 lizdas expansion hub
        sm2 = hwMap.get(DcMotorEx.class, "svK");  // 1 lizdas expansion hub
        pad = hwMap.get(DcMotor.class, "pad");  // 2 lizdas expansion hub
        pem = hwMap.get(DcMotor.class, "pem");  // 3 lizdas expansion hub
        sm1.setDirection(DcMotor.Direction.REVERSE);
        pad.setDirection(DcMotor.Direction.REVERSE);

    }
    public class autougnis implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                sm1.setPower(targetVelocity);
                sm2.setPower(targetVelocity);
                sleep(400);
                sm1.setPower(targetVelocity);
                sm2.setPower(targetVelocity);
                pad.setPower(0.55);
                pem.setPower(-0.5);
                sleep(600);
                sm1.setPower(0);
                sm2.setPower(0);
                pad.setPower(0);
                pem.setPower(0);
                initialized = true;
            }
            return false;
        }
    }
    public  Action autougnis() {
        return new Šaudyklė.autougnis();
    }
    public class autoatgal1 implements Action {
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
    public  Action autoatgal1() {
        return new Šaudyklė.autoatgal1();
    }
    public class autoatgal0 implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                pem.setPower(0);
                pad.setPower(0);
                sm1.setPower(0);
                sm2.setPower(0);
                initialized = true;
            }
            return false;
        }
    }
    public  Action autoatgal0() {
        return new Šaudyklė.autoatgal0();
    }
    public void teleugnis(double sp){
        sm1.setPower(sp);
        sm2.setPower(sp);
        sleep(400);
        sm1.setPower(sp);
        sm2.setPower(sp);
        pad.setPower(0.55);
        pem.setPower(-0.5);
        sleep(600);
        sm1.setPower(0);
        sm2.setPower(0);
        pad.setPower(0);
        pem.setPower(0);
    }
    public void teleatgal1(){
        pem.setPower(0.7);
        pad.setPower(0.7);
        sm1.setPower(-0.6);
        sm2.setPower(-0.6);
    }
    public void teleatgal0(){
        pem.setPower(0);
        pad.setPower(0);
        sm1.setPower(0);
        sm2.setPower(0);
    }


}
