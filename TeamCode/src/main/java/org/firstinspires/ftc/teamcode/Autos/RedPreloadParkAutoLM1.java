package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Red preload auto: shoot 4 from the RedAutoLM1 start, park in the loading zone. Logic is in PreloadParkAutoLM1.
@Autonomous(name = "RedPreloadParkAutoLM1", group = "LM1", preselectTeleOp = "LimelightOnlyTeleopRedLM1")
public class RedPreloadParkAutoLM1 extends PreloadParkAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }
}
