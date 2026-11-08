package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Red alliance Pedro auto. Route and logic are in PedroAutoLM1. Pairs with RedTeleopLM1.
@Autonomous(name = "RedAutoLM1", group = "LM1", preselectTeleOp = "RedTeleopLM1")
public class RedAutoLM1 extends PedroAutoLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.RED;
    }
}
