package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;

// Blue alliance teleop with hive auto-aim. All the logic is in AimTeleopLM1.
@TeleOp(name = "BlueTeleopLM1", group = "LM1")
public class BlueTeleopLM1 extends AimTeleopLM1 {
    @Override
    protected FieldLM1.Alliance alliance() {
        return FieldLM1.Alliance.BLUE;
    }
}
