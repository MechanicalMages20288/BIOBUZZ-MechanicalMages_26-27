package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

/*
 * Red 2x Flower auto WITH the hive check: same as RedLM12xFlowerAuto, but on reaching the far
 * shot spot for the first time the Limelight checks that the cell is tipped up (its tags are
 * visible). Tipped up -> shoot right away and carry on as usual. Not tipped -> drive down the
 * lane between the hives, shoot the preload cell from the start spot, park up the side.
 * Logic is in PedroAutoLM1.
 */
@Autonomous(name = "RedHiveCheck2xFlowerAutoLM1", group = "LM1", preselectTeleOp = "RedTeleopLM1")
public class RedHiveCheck2xFlowerAutoLM1 extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }

    @Override
    protected boolean sideFlowerInsteadOfWall() {
        return true;
    }

    @Override
    protected boolean useHiveCheck() {
        return true;
    }
}
