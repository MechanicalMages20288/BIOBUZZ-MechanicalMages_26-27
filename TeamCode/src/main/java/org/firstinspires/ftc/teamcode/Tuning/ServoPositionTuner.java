package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Finds the UP and DOWN positions for a servo, and how big/fast its "wiggle" should be.
 * Nothing else on the robot moves. When you're happy, copy the numbers from the bottom of
 * the telemetry into the match teleop.
 *
 * WHAT THE SERVO DOES
 *   UP / DOWN    two fixed positions.
 *   WIGGLE       only while DOWN: it moves a small amount toward UP and back, over and over,
 *                then returns to DOWN when you let go.
 *
 * GAMEPAD 1 CONTROLS
 *   A                    go UP
 *   B                    go DOWN
 *   X (hold)             WIGGLE (only works while DOWN)
 *   D-pad up / down      pick which value to edit (marked with >)
 *   D-pad right / left   increase / decrease that value
 *   Right bumper (hold)  fine steps (10x smaller) while pressing d-pad right/left
 *
 * Editing UP or DOWN moves the servo there right away, so you can watch it.
 * Servo positions go from 0.0 to 1.0.
 */
@TeleOp(name = "Servo Position Tuner", group = "Tuning")
public class ServoPositionTuner extends OpMode {

    // Name of the servo in the robot configuration.
    public static String SERVO_NAME = "servo";

    // Starting values. Change these once you've found good ones, so the tuner starts there.
    public static double upPosition = 0.8;
    public static double downPosition = 0.2;
    public static double wiggleAmount = 0.05;   // how far toward UP each wiggle goes (0-1 scale)
    public static double wigglesPerSecond = 4;  // full down-and-back wiggles per second

    // Step sizes for one d-pad tap (right bumper makes them 10x smaller).
    final double POSITION_STEP = 0.01;
    final double WIGGLE_SPEED_STEP = 0.5;

    private Servo servo;
    private final ElapsedTime wiggleTimer = new ElapsedTime();
    private boolean isDown = true;
    private boolean wiggling = false;

    // Which value the d-pad edits: 0 = up, 1 = down, 2 = wiggle amount, 3 = wiggle speed.
    private int selected = 0;
    private final String[] NAMES = {"Up position", "Down position", "Wiggle amount", "Wiggles per second"};

    // Last loop's buttons, so one tap = one change.
    private boolean prevUp, prevDown, prevRight, prevLeft;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, SERVO_NAME);
        telemetry.addData("Status", "Initialized. Servo moves to DOWN when you press START.");
    }

    @Override
    public void start() {
        isDown = true;
        servo.setPosition(downPosition);
    }

    @Override
    public void loop() {
        // ---- Up / down ----
        if (gamepad1.a) {
            isDown = false;
        }
        if (gamepad1.b) {
            isDown = true;
        }

        // ---- Pick which value to edit ----
        if (gamepad1.dpad_up && !prevUp) {
            selected = (selected + NAMES.length - 1) % NAMES.length;
        }
        if (gamepad1.dpad_down && !prevDown) {
            selected = (selected + 1) % NAMES.length;
        }

        // ---- Change it ----
        int direction = 0;
        if (gamepad1.dpad_right && !prevRight) direction = 1;
        if (gamepad1.dpad_left && !prevLeft) direction = -1;
        if (direction != 0) {
            double scale = gamepad1.right_bumper ? 0.1 : 1.0;
            switch (selected) {
                case 0:
                    upPosition = clamp(upPosition + direction * POSITION_STEP * scale, 0, 1);
                    isDown = false; // show the new UP position
                    break;
                case 1:
                    downPosition = clamp(downPosition + direction * POSITION_STEP * scale, 0, 1);
                    isDown = true;  // show the new DOWN position
                    break;
                case 2:
                    wiggleAmount = clamp(wiggleAmount + direction * POSITION_STEP * scale, 0, 1);
                    break;
                case 3:
                    wigglesPerSecond = clamp(wigglesPerSecond + direction * WIGGLE_SPEED_STEP * scale, 0.5, 20);
                    break;
            }
        }
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevRight = gamepad1.dpad_right;
        prevLeft = gamepad1.dpad_left;

        // ---- Move the servo ----
        boolean wantWiggle = isDown && gamepad1.x;
        if (wantWiggle && !wiggling) {
            wiggleTimer.reset(); // start each wiggle from the DOWN position
        }
        wiggling = wantWiggle;

        double target;
        if (!isDown) {
            target = upPosition;
        } else if (wiggling) {
            target = wigglePosition();
        } else {
            target = downPosition;
        }
        servo.setPosition(target);

        // ---- Telemetry ----
        telemetry.addData("State", !isDown ? "UP" : (wiggling ? "DOWN + WIGGLING" : "DOWN"));
        telemetry.addData("Servo position", "%.3f", target);
        telemetry.addLine();
        double[] values = {upPosition, downPosition, wiggleAmount, wigglesPerSecond};
        for (int i = 0; i < NAMES.length; i++) {
            telemetry.addData((i == selected ? "> " : "   ") + NAMES[i], "%.3f", values[i]);
        }
        telemetry.addLine();
        telemetry.addLine("A up | B down | hold X wiggle | d-pad edit | RB fine");
        telemetry.addLine();
        telemetry.addLine("Copy into the teleop:");
        telemetry.addLine(String.format("UP = %.3f, DOWN = %.3f, WIGGLE = %.3f, WIGGLE_HZ = %.1f",
                upPosition, downPosition, wiggleAmount, wigglesPerSecond));
    }

    /*
     * Wiggle: first half of each cycle at DOWN, second half moved WIGGLE toward UP.
     * It always moves toward UP (never past DOWN), in case DOWN is up against something.
     */
    private double wigglePosition() {
        double towardUp = Math.signum(upPosition - downPosition);
        double cycle = (wiggleTimer.seconds() * wigglesPerSecond) % 1.0;
        double offset = cycle < 0.5 ? 0 : wiggleAmount * towardUp;
        return clamp(downPosition + offset, 0, 1);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
