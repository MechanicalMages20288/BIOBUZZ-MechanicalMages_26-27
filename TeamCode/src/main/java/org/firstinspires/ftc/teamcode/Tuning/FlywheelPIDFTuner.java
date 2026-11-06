package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Flywheel PIDF tuner. Uses ONLY the launcher motor; feed balls in by hand.
 * Adjust the launcher's P, I, D, F and target velocity live with gamepad 1 while watching the
 * speed on telemetry, then copy the final numbers into the teleop and auto (the
 * setPIDFCoefficients(...) line and the target velocity).
 *
 * All velocities are in encoder ticks per second, same as the teleop.
 *
 * GAMEPAD 1 CONTROLS
 *   A               flywheel on / off (speed-controlled, at the target velocity)
 *   B               "max speed test" on / off: full power with no speed control.
 *                   Use this first to measure top speed; it shows a suggested F.
 *   D-pad up/down   choose which value to change (P, I, D, F, Target)
 *   D-pad right/left  raise / lower the chosen value by the step size
 *   Y / X           step size x10 / /10
 *   Right trigger   raise target velocity by the Target step (default 25)
 *   Left trigger    lower target velocity by the Target step
 *   Back            reset the readings
 *
 * SHOT DETECTION
 *   With the flywheel on and at speed, a shot is detected when the speed drops more than
 *   SHOT_DROP below target. The tuner records how far it dropped ("dip") and how long it
 *   took to get back within TOLERANCE ("recovery").
 *
 * TUNING STEPS
 *   1. Check "Actual" changes when the flywheel spins. If it stays at 0, fix the encoder
 *      cable first; tuning can't work without it.
 *   2. Press B, wait for the speed to settle, note "Max seen" and "Suggested F". Press B again.
 *   3. Set P, I, D to 0 and F to the suggested value. Press A. Actual should land close to
 *      target. Nudge F until it does.
 *   4. Raise P until the speed recovers quickly after shots without bouncing around the
 *      target. If it bounces or the motor buzzes, lower P.
 *   5. Feed real balls by hand. Watch "Last shot dip" and "Last recovery". Smaller is better.
 */

@TeleOp(name = "Flywheel PIDF Tuner", group = "Tuning")
public class FlywheelPIDFTuner extends OpMode {

    // How close (ticks/s) actual must be to target to count as "at speed".
    final double TOLERANCE = 50;

    // How far (ticks/s) below target the speed must drop to count as a shot.
    final double SHOT_DROP = 100;

    // Values being tuned. They start at the teleop's current values.
    double p = 40;
    double i = 0;
    double d = 0;
    double f = 14.4;
    double targetVelocity = 1650;

    // Which value the d-pad changes: 0 = P, 1 = I, 2 = D, 3 = F, 4 = Target.
    int selected = 0;
    final String[] NAMES = {"P", "I", "D", "F", "Target"};

    // Step size for each value, changed with Y / X.
    double[] steps = {1, 0.1, 0.1, 0.1, 25};

    boolean flywheelOn = false;
    boolean maxSpeedTest = false;

    // Readings to judge the tuning.
    double maxSeen = 0;
    double lowestDuringShot = Double.MAX_VALUE;
    double lastDip = 0;
    double lastRecoverySeconds = -1;
    int shotCount = 0;
    boolean inShot = false;
    boolean wasAtSpeed = false;
    private final ElapsedTime shotTimer = new ElapsedTime();

    // Last loop's button states, so one press does one thing.
    boolean prevA, prevB, prevX, prevY, prevUp, prevDown, prevLeft, prevRight, prevBack;
    boolean prevRightTrigger, prevLeftTrigger;

    private DcMotorEx launcher;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        // Reverse in code (not by swapping wires) so the motor and its encoder stay matched.
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPIDF();

        telemetry.addLine("Flywheel PIDF Tuner ready. Press START, then A to spin up.");
    }

    @Override
    public void loop() {
        handleButtons();

        if (maxSpeedTest) {
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(1);
        } else {
            if (launcher.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                applyPIDF();
            }
            launcher.setVelocity(flywheelOn ? targetVelocity : 0);
        }

        double actual = launcher.getVelocity();
        maxSeen = Math.max(maxSeen, actual);

        trackShots(actual);
        showTelemetry(actual);
    }

    @Override
    public void stop() {
        launcher.setPower(0);
    }

    void handleButtons() {
        if (gamepad1.a && !prevA) {
            flywheelOn = !flywheelOn;
            maxSpeedTest = false;
        }
        if (gamepad1.b && !prevB) {
            maxSpeedTest = !maxSpeedTest;
            flywheelOn = false;
            if (!maxSpeedTest) {
                launcher.setPower(0);
            }
        }
        if (gamepad1.dpad_up && !prevUp) {
            selected = (selected + NAMES.length - 1) % NAMES.length;
        }
        if (gamepad1.dpad_down && !prevDown) {
            selected = (selected + 1) % NAMES.length;
        }
        if (gamepad1.dpad_right && !prevRight) {
            changeSelected(steps[selected]);
        }
        if (gamepad1.dpad_left && !prevLeft) {
            changeSelected(-steps[selected]);
        }
        if (gamepad1.y && !prevY) {
            steps[selected] *= 10;
        }
        if (gamepad1.x && !prevX) {
            steps[selected] /= 10;
        }
        // Triggers adjust the target velocity directly (one step per pull), whatever is selected.
        boolean rightTrigger = gamepad1.right_trigger > 0.5;
        boolean leftTrigger = gamepad1.left_trigger > 0.5;
        if (rightTrigger && !prevRightTrigger) {
            targetVelocity += steps[4];
        }
        if (leftTrigger && !prevLeftTrigger) {
            targetVelocity = Math.max(0, targetVelocity - steps[4]);
        }
        prevRightTrigger = rightTrigger;
        prevLeftTrigger = leftTrigger;

        if (gamepad1.back && !prevBack) {
            maxSeen = 0;
            lastDip = 0;
            lastRecoverySeconds = -1;
            shotCount = 0;
            inShot = false;
        }

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;
        prevRight = gamepad1.dpad_right;
        prevBack = gamepad1.back;
    }

    void changeSelected(double amount) {
        switch (selected) {
            case 0: p = Math.max(0, p + amount); break;
            case 1: i = Math.max(0, i + amount); break;
            case 2: d = Math.max(0, d + amount); break;
            case 3: f = Math.max(0, f + amount); break;
            case 4: targetVelocity = Math.max(0, targetVelocity + amount); break;
        }
        if (selected <= 3) {
            applyPIDF();
        }
    }

    void applyPIDF() {
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    /*
     * A shot starts when the flywheel was at speed and then drops more than SHOT_DROP below
     * target. It ends when the speed is back within TOLERANCE. We record the lowest speed
     * during the shot (dip) and the time from the drop to recovery.
     */
    void trackShots(double actual) {
        if (!flywheelOn) {
            inShot = false;
            wasAtSpeed = false;
            return;
        }

        boolean atSpeed = Math.abs(targetVelocity - actual) <= TOLERANCE;

        if (!inShot && wasAtSpeed && actual < targetVelocity - SHOT_DROP) {
            inShot = true;
            lowestDuringShot = actual;
            shotTimer.reset();
        }
        if (inShot) {
            lowestDuringShot = Math.min(lowestDuringShot, actual);
            if (atSpeed) {
                inShot = false;
                shotCount++;
                lastDip = targetVelocity - lowestDuringShot;
                lastRecoverySeconds = shotTimer.seconds();
            }
        }

        if (atSpeed) {
            wasAtSpeed = true;
        }
    }

    void showTelemetry(double actual) {
        String mode = maxSpeedTest ? "MAX SPEED TEST (full power)" : (flywheelOn ? "ON" : "OFF");
        telemetry.addData("Flywheel", mode);
        telemetry.addData("Target", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Actual", "%.0f ticks/s", actual);
        telemetry.addData("Error", "%.0f ticks/s", targetVelocity - actual);
        telemetry.addData("At speed", Math.abs(targetVelocity - actual) <= TOLERANCE ? "YES" : "no");
        telemetry.addLine();

        telemetry.addLine("--- PIDF (d-pad up/down to choose, left/right to change) ---");
        double[] values = {p, i, d, f, targetVelocity};
        for (int k = 0; k < NAMES.length; k++) {
            String arrow = (k == selected) ? ">> " : "   ";
            telemetry.addData(arrow + NAMES[k], "%.3f   (step %.3f)", values[k], steps[k]);
        }
        telemetry.addLine();

        telemetry.addLine("--- Readings (Back to reset) ---");
        telemetry.addData("Max seen", "%.0f ticks/s", maxSeen);
        telemetry.addData("Suggested F", maxSeen > 0 ? String.format("%.2f  (32767 / max)", 32767 / maxSeen) : "press B to measure");
        telemetry.addData("Shots detected", shotCount);
        telemetry.addData("Last shot dip", "%.0f ticks/s", lastDip);
        telemetry.addData("Last recovery", inShot ? "recovering..."
                : (lastRecoverySeconds < 0 ? "feed a ball in by hand" : String.format("%.2f s", lastRecoverySeconds)));
        telemetry.addLine();

        telemetry.addLine("Copy into teleop/auto:");
        telemetry.addLine(String.format("new PIDFCoefficients(%.3f, %.3f, %.3f, %.3f)", p, i, d, f));
        telemetry.update();
    }
}
