package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Blue alliance teleop with Limelight-based position. Logic is in LimelightAimTeleopLM1.
@TeleOp(name = "LimelightTeleopBlueLM1", group = "LM1")
public class LimelightTeleopBlueLM1 extends LimelightAimTeleopLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }
}
