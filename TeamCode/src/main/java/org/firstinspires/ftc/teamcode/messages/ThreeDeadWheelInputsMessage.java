package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class ThreeDeadWheelInputsMessage {
    public long timestamp;
    public PositionVelocityPair krded;
    public PositionVelocityPair dsded;
    public PositionVelocityPair vdded;

    public ThreeDeadWheelInputsMessage(PositionVelocityPair krded, PositionVelocityPair dsded, PositionVelocityPair vdded) {
        this.timestamp = System.nanoTime();
        this.krded = krded;
        this.dsded = dsded;
        this.vdded = vdded;
    }
}