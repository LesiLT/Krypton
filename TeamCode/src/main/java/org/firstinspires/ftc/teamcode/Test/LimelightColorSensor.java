
package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight Color Sensor", group = "Test")
public class LimelightColorSensor extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        // 0 yra geltoniem kamuoliam
        limelight.pipelineSwitch(0);

        // startas rezultatams
        limelight.start();

        telemetry.addLine("Limelight ijungtas");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // sarasas taikiniu
                List<LLResultTypes.ColorResult> colors =
                        result.getColorResults();

                telemetry.addData("Taikiniai", colors.size());

                for (int i = 0; i < colors.size(); i++) {

                    LLResultTypes.ColorResult color = colors.get(i);

                    telemetry.addData("Taikinys " + i,
                            "X: %.2f | Y: %.2f",
                            color.getTargetXDegrees(),
                            color.getTargetYDegrees());

                    telemetry.addData("Taikinio plotas " + i,
                            "%.2f%%",
                            color.getTargetArea());

                    telemetry.addData("Pikselio pozicija " + i,
                            "X: %.1f | Y: %.1f",
                            color.getTargetXPixels(),
                            color.getTargetYPixels());
                }

            } else {
                telemetry.addLine("Nieko nerasta");
            }

            telemetry.addData("Prisijungta", limelight.isConnected());

            telemetry.update();

            sleep(20);
        }

        limelight.stop();
    }
}
