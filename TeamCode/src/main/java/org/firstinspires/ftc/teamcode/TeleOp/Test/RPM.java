package org.firstinspires.ftc.teamcode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp (name = "RPM")
public class RPM extends LinearOpMode {
    private DcMotorEx shooterLeft, shooterRight;
    DcMotor pem;
    DcMotorEx pad;

    // Motor specs
    private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
    private static final int MAX_RPM = 6000;
    private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

    double sp=0.5;
    // Start at ~70% power
    private double targetVelocity = MAX_TICKS_PER_SEC * 0.7;   ///derinsimes
    double pvelo =targetVelocity;

    @Override
    public void runOpMode() throws InterruptedException {
        pem = hardwareMap.get(DcMotor.class, "pem" ); //paėmimas
        pad = hardwareMap.get(DcMotorEx.class, "pad" ); //padavimas
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "svK"); //Kairys
        shooterRight = hardwareMap.get(DcMotorEx.class, "svD"); //Dešinys


        shooterRight.setDirection(DcMotor.Direction.REVERSE);

        // PIDF coefficients tuned for 6000 RPM
        shooterLeft.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
        shooterRight.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
        pad.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);


        waitForStart();

        while (opModeIsActive()) {

            // --- Gamepad controls ---
            if (gamepad1.dpad_up) {
                targetVelocity += 50;  // increase by 50 ticks/sec
            }
            if (gamepad1.dpad_down) {
                targetVelocity -= 50;  // decrease by 50 ticks/sec
            }

                if (gamepad1.circle) {
                    targetVelocity = 0;
                    pvelo = 0;// stop shooter
                }

                    if (gamepad1.square) {
                        targetVelocity = MAX_TICKS_PER_SEC; // full speed
                    }
                    //shooterLeft.setPower(0.6);
                    //shooterRight.setPower(0.6);
                        if (gamepad1.dpad_left) {
                            pvelo -= 10;
                        }
                        if (gamepad1.dpad_right) {
                            pvelo += 10;
                        }

                        if (gamepad1.right_bumper) {
                            pem.setPower(0.5);
                        }
                        else if (!gamepad1.right_bumper) {
                            pem.setPower(0);
                        }


                        // Clamp target
                        targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));

                        // Apply velocity
                        shooterLeft.setVelocity(targetVelocity);
                        shooterRight.setVelocity(targetVelocity);
                        pad.setVelocity(pvelo);

                        // Telemetry for driver feedback
                        telemetry.addData("Target Velocity", pvelo);
                        telemetry.addData("Target Velocity", targetVelocity);
                        telemetry.addData("Target RPM", (targetVelocity / CPR) * 60);
                        telemetry.addData("Left Vel", shooterLeft.getVelocity());
                        telemetry.addData("Right Vel", shooterRight.getVelocity());
                        telemetry.update();





        }
    }}
