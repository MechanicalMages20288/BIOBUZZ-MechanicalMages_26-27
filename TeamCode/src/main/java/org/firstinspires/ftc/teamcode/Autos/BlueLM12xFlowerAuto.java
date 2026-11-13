package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

/*
 * Blue 2x Flower auto: same as BlueAutoLM1, but instead of the corner wall intake it intakes
 * from the alliance side flower (red wall, y ~46 on red; mirrored for blue),
 * using the same flower steps as the top flower. Route and logic are in PedroAutoLM1.
 */
@Autonomous(name = "BlueLM12xFlowerAuto", group = "LM1", preselectTeleOp = "BlueTeleopLM1")
public class BlueLM12xFlowerAuto extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }

    @Override
    protected boolean sideFlowerInsteadOfWall() {
        return true;
    }
}
