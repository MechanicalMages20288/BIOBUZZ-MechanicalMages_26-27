package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Backup", group = "StarterBot")
public class BiobuzzStarterBotBackup extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Status", "Backup Auto Initialized (Idle)");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running (No Action)");
        telemetry.update();
    }
}
