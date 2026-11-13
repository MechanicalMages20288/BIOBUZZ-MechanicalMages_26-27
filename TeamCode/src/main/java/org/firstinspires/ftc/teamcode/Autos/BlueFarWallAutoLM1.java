package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Blue far-wall auto: waits for our far-side cell's tags, shoots 4, parks. Logic is in FarWallTagAutoLM1.
@Autonomous(name = "BlueFarWallAutoLM1", group = "LM1", preselectTeleOp = "LimelightOnlyTeleopBlueLM1")
public class BlueFarWallAutoLM1 extends FarWallTagAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }
}
