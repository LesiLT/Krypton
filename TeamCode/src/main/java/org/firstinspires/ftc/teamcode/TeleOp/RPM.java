package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp (name = "RPM")
public class RPM extends LinearOpMode {
        private DcMotorEx shooterLeft, shooterRight;
        DcMotor pad;

        // Motor specs
        private static final int CPR = 28;             // encoder counts per rev (GoBILDA 6000RPM)
        private static final int MAX_RPM = 6000;
        private static final int MAX_TICKS_PER_SEC = (MAX_RPM / 60) * CPR;  // ~2800

        // Start at ~70% power
        private double targetVelocity = MAX_TICKS_PER_SEC * 0.7;   ///derinsimes

        @Override
        public void runOpMode() throws InterruptedException {
            pad = hardwareMap.get(DcMotor.class, "pad" ); //padavimas
            shooterLeft  = hardwareMap.get(DcMotorEx.class, "svK"); //Kairys
            shooterRight = hardwareMap.get(DcMotorEx.class, "svD"); //Dešinys

            shooterRight.setDirection(DcMotor.Direction.REVERSE);

            // PIDF coefficients tuned for 6000 RPM
            shooterLeft.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);
            shooterRight.setVelocityPIDFCoefficients(0.01, 0.0, 0.001, 11.7);

            waitForStart();

            while (opModeIsActive()) {

                // --- Gamepad controls ---
                if (gamepad1.dpad_up) {
                    targetVelocity += 50;  // increase by 50 ticks/sec
                }
                if (gamepad1.dpad_down) {
                    targetVelocity -= 50;  // decrease by 50 ticks/sec
                }
                if (gamepad1.a) {
                    targetVelocity = 0;    // stop shooter
                }
                if (gamepad1.b) {
                    targetVelocity = MAX_TICKS_PER_SEC; // full speed
                }
                //shooterLeft.setPower(0.6);
                //shooterRight.setPower(0.6);
                if(gamepad1.left_bumper){
                    pad.setPower(0.4);
                }

                   if (gamepad1.left_stick_x >0 || gamepad1.left_stick_x <0){
                       pad.setPower(gamepad1.left_stick_x);
                   }
                    else pad.setPower(0);


                // Clamp target
                targetVelocity = Math.max(0, Math.min(MAX_TICKS_PER_SEC, targetVelocity));

                // Apply velocity
                shooterLeft.setVelocity(targetVelocity);
                shooterRight.setVelocity(targetVelocity);

                // Telemetry for driver feedback
                telemetry.addData("Target Velocity", targetVelocity);
                telemetry.addData("Target RPM", (targetVelocity / CPR) * 60);
                telemetry.addData("Left Vel", shooterLeft.getVelocity());
                telemetry.addData("Right Vel", shooterRight.getVelocity());
                telemetry.update();
            }
        }
    }

