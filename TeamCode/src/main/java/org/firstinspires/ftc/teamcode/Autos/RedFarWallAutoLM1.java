package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Red far-wall auto: waits for our far-side cell's tags, shoots 4, parks. Logic is in FarWallTagAutoLM1.
@Autonomous(name = "RedFarWallAutoLM1", group = "LM1", preselectTeleOp = "LimelightOnlyTeleopRedLM1")
public class RedFarWallAutoLM1 extends FarWallTagAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }
}
