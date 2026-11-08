package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Blue alliance Pedro auto: the red route rotated 180 degrees. Pairs with BlueTeleopLM1.
@Autonomous(name = "BlueAutoLM1", group = "LM1", preselectTeleOp = "BlueTeleopLM1")
public class BlueAutoLM1 extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }
}
