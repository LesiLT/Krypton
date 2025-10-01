package org.firstinspires.ftc.teamcode.Kamera;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilLibrary {
    public static AprilTagLibrary getSmallLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(
                        20,
                        "melynas",
                        16.5,
                        new VectorF(6 ,1 ,10),
                        DistanceUnit.CM,
                        Quaternion.identityQuaternion()
                )
                .addTag(
                        24,
                        "raudonas",
                        16.5,
                        new VectorF(6,6,10),
                        DistanceUnit.CM,
                        Quaternion.identityQuaternion()
                )
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

    }


    }

