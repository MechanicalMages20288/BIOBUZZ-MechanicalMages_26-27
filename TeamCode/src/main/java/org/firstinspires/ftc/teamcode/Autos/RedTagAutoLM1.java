package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

/*
 * Red corner auto WITH the tag check: same as RedAutoLM1, but after the preloads it watches the
 * cell's tags. Still visible after 3 s -> corner intake, back to start, shoot that cell again,
 * then top flower and far shot. Gone -> RedAutoLM1's route. Logic is in PedroAutoLM1.
 */
@Autonomous(name = "RedTagAutoLM1", group = "LM1", preselectTeleOp = "RedTeleopLM1")
public class RedTagAutoLM1 extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }

    @Override
    protected boolean useTagCheck() {
        return true;
    }
}
