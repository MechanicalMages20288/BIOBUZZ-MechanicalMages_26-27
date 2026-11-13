package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

/*
 * Red 2x Flower auto WITH the tag check: same as RedLM12xFlowerAuto, but after the preloads it
 * watches the cell's tags. Still visible after 3 s -> side flower, back to start, shoot that cell
 * again, then top flower and far shot. Gone -> RedLM12xFlowerAuto's route. Logic is in PedroAutoLM1.
 */
@Autonomous(name = "RedTag2xFlowerAutoLM1", group = "LM1", preselectTeleOp = "RedTeleopLM1")
public class RedTag2xFlowerAutoLM1 extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }

    @Override
    protected boolean sideFlowerInsteadOfWall() {
        return true;
    }

    @Override
    protected boolean useTagCheck() {
        return true;
    }
}
