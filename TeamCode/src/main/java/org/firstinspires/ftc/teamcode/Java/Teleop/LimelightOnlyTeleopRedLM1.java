package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Red alliance teleop, position from the Limelight ONLY (no odometry). Logic is in LimelightOnlyAimTeleopLM1.
@TeleOp(name = "LimelightOnlyTeleopRedLM1", group = "LM1")
public class LimelightOnlyTeleopRedLM1 extends LimelightOnlyAimTeleopLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }
}
