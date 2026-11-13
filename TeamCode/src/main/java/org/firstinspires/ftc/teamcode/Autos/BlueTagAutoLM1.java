package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

/*
 * Blue corner auto WITH the tag check: same as BlueAutoLM1, but after the preloads it watches the
 * cell's tags. Still visible after 3 s -> corner intake, back to start, shoot that cell again,
 * then top flower and far shot. Gone -> BlueAutoLM1's route. Logic is in PedroAutoLM1.
 */
@Autonomous(name = "BlueTagAutoLM1", group = "LM1", preselectTeleOp = "BlueTeleopLM1")
public class BlueTagAutoLM1 extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }

    @Override
    protected boolean useTagCheck() {
        return true;
    }
}
