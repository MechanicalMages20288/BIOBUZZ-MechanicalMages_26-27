package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Blue alliance teleop, position from the Limelight ONLY (no odometry). Logic is in LimelightOnlyAimTeleopLM1.
@TeleOp(name = "LimelightOnlyTeleopBlueLM1", group = "LM1")
public class LimelightOnlyTeleopBlueLM1 extends LimelightOnlyAimTeleopLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }
}
