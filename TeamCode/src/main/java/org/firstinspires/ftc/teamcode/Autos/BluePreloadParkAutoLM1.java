package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Blue preload auto: shoot 4 from the BlueAutoLM1 start, park in the loading zone. Logic is in PreloadParkAutoLM1.
@Autonomous(name = "BluePreloadParkAutoLM1", group = "LM1", preselectTeleOp = "LimelightOnlyTeleopBlueLM1")
public class BluePreloadParkAutoLM1 extends PreloadParkAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }
}
