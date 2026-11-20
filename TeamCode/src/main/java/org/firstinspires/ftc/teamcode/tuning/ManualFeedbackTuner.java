package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 10;

    @Override
    public void runOpMode() {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            
            if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set!");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        }
    }
}
