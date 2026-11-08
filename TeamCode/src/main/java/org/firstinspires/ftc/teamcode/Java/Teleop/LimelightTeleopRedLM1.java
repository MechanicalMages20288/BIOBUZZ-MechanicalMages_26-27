package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Red alliance teleop with Limelight-based position. Logic is in LimelightAimTeleopLM1.
@TeleOp(name = "LimelightTeleopRedLM1", group = "LM1")
public class LimelightTeleopRedLM1 extends LimelightAimTeleopLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }
}
