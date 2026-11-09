package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Finds a servo's UP and DOWN positions. The servo goes to START_POSITION when you press START
 * (not at INIT, so it never jumps while you're setting the robot down). Then nudge it with
 * gamepad 1 and read the position off the telemetry.
 *
 * GAMEPAD 1 CONTROLS
 *   D-pad up / down      +/- 0.01  (big steps)
 *   D-pad right / left   +/- 0.001 (fine steps)
 *   A                    save the current position as UP
 *   B                    save the current position as DOWN
 *   X / Y                jump to the saved DOWN / UP (to check them)
 *
 * Servo positions go from 0.0 to 1.0.
 */
@TeleOp(name = "Servo Position Tuner", group = "Tuning")
public class ServoPositionTuner extends OpMode {

    // Name of the servo in the robot configuration.
    public static String SERVO_NAME = "servo";

    // Where the servo goes at START. Pick a position the mechanism can reach safely.
    public static double START_POSITION = 0.5;

    final double BIG_STEP = 0.01;
    final double FINE_STEP = 0.001;

    private Servo servo;
    private double position = START_POSITION;
    private double savedUp = Double.NaN;
    private double savedDown = Double.NaN;

    // Last loop's buttons, so one tap = one step.
    private boolean prevUp, prevDown, prevRight, prevLeft;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        telemetry.addData("Status", "Ready. Servo moves to %.3f when you press START.", START_POSITION);
    }

    @Override
    public void start() {
        position = START_POSITION;
        servo.setPosition(position);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !prevUp) position += BIG_STEP;
        if (gamepad1.dpad_down && !prevDown) position -= BIG_STEP;
        if (gamepad1.dpad_right && !prevRight) position += FINE_STEP;
        if (gamepad1.dpad_left && !prevLeft) position -= FINE_STEP;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevRight = gamepad1.dpad_right;
        prevLeft = gamepad1.dpad_left;

        if (gamepad1.a) savedUp = position;
        if (gamepad1.b) savedDown = position;
        if (gamepad1.y && !Double.isNaN(savedUp)) position = savedUp;
        if (gamepad1.x && !Double.isNaN(savedDown)) position = savedDown;

        position = Math.max(0, Math.min(1, position));
        servo.setPosition(position);

        telemetry.addData("Servo position", "%.3f", position);
        telemetry.addData("Saved UP", Double.isNaN(savedUp) ? "not set (press A)" : String.format("%.3f", savedUp));
        telemetry.addData("Saved DOWN", Double.isNaN(savedDown) ? "not set (press B)" : String.format("%.3f", savedDown));
        telemetry.addLine();
        telemetry.addLine("D-pad up/down 0.01 | right/left 0.001");
        telemetry.addLine("A save UP | B save DOWN | Y go UP | X go DOWN");
    }
}
