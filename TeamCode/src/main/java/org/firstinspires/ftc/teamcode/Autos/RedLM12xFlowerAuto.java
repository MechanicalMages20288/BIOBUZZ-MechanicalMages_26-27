package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

/*
 * Red 2x Flower auto: same as RedAutoLM1, but instead of the corner wall intake it intakes
 * from the alliance side flower (red wall, y ~46),
 * using the same flower steps as the top flower. Route and logic are in PedroAutoLM1.
 */
@Autonomous(name = "RedLM12xFlowerAuto", group = "LM1", preselectTeleOp = "RedTeleopLM1")
public class RedLM12xFlowerAuto extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }

    @Override
    protected boolean sideFlowerInsteadOfWall() {
        return true;
    }
}
