package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name="LimelightAprilTag")
public class LimeLightAprilTag0 extends LinearOpMode {

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //April tag'am (nemodifikuota)
        limelight.pipelineSwitch(1);

        limelight.start();
        telemetry.addLine("Limelight ijungtas");
        telemetry.update();

        waitForStart();
        while(!opModeIsActive())
        {
            LLResult result = limelight.getLatestResult();
            if(result != null && result.isValid())
            {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for(int i=0; i<tags.size(); i++)
                {
                    LLResultTypes.FiducialResult tag = tags.get(i);
                    telemetry.addData("ID", tag.getFiducialId());
                    telemetry.addData("tx", tag.getTargetXDegrees());
                    telemetry.addData("ty", tag.getTargetYDegrees());
                    telemetry.addData("Area", tag.getTargetArea());
                }
            }
            else
            {
                telemetry.addLine("Nerasta");
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
