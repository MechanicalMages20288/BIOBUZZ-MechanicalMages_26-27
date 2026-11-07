package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;
import org.firstinspires.ftc.teamcode.Java.Field.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Shared code for RedTeleopLM1 and BlueTeleopLM1: everything in BasicTeleopLM1, plus hive
 * auto-aim using the Pinpoint. The Red/Blue classes only say which alliance they are.
 *
 * GAMEPAD 1 (driver) EXTRA CONTROLS
 *   A            tap: auto-aim ON. The robot turns so its BACK faces your hive's active cell
 *                and holds it there. Moving any stick (or tapping A again) turns it OFF.
 *   X            tap: switch to the OTHER cell of your hive (override, e.g. after it tips).
 *                Tap again to go back to automatic.
 *   D-pad left   relocalize: robot pushed into your AUDIENCE-side corner (see FieldLM1).
 *   D-pad right  relocalize: robot pushed into your FAR-side corner.
 *
 * WHICH CELL: automatic by field half. Robot on the audience half (y < 72) aims at the
 * audience-side cell, on the far half at the far-side cell. X flips that choice.
 *
 * STARTING POSITION: taken from the auto that just ran (PoseStorage). If no auto ran for this
 * alliance, it assumes the robot is at the auto start spot; use a corner button to fix it.
 */
public abstract class AimTeleopLM1 extends BasicTeleopLM1 {

    // Turning power per radian of heading error, and limits on the turning power.
    public static double AIM_KP = 1.2;
    public static double AIM_MAX_TURN = 0.6;
    public static double AIM_MIN_TURN = 0.06; // small push so it doesn't stall just short
    public static double AIM_TOLERANCE_DEG = 2; // within this, it's "LOCKED ON"

    // Any stick pushed past this cancels auto-aim and gives control back to the driver.
    public static double STICK_CANCEL = 0.15;

    // Which alliance this teleop is for.
    protected abstract FieldLM1.Alliance alliance();

    private PinpointLocalizer localizer;
    private Pose startPose;
    private String poseSource = "";

    private boolean aiming = false;
    private boolean flipTarget = false;
    private Pose activeCell = null;
    private double headingErrorRad = 0;

    private boolean prevA, prevX, prevDpadLeft, prevDpadRight;

    @Override
    public void init() {
        super.init();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        if (PoseStorage.lastPose != null && PoseStorage.alliance == alliance()) {
            startPose = PoseStorage.lastPose;
            poseSource = "from auto";
        } else {
            startPose = FieldLM1.autoStartPose(alliance());
            poseSource = "default auto start spot (no auto ran)";
        }
        localizer.setPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Start pose", "%s  (%s)", formatPose(startPose), poseSource);
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        // Set again at START in case the Pinpoint was still resetting during INIT.
        localizer.setPose(startPose);
        PoseStorage.clear();
    }

    @Override
    public void loop() {
        localizer.update();
        handleDriverButtons();

        // BasicTeleopLM1 drives and runs the shooter; it calls turnCommand() below for turning.
        super.loop();

        Pose pose = localizer.pose();
        telemetry.addLine();
        telemetry.addData("Auto-aim", aiming ? (isLockedOn() ? "LOCKED ON" : "TURNING") : "off (tap A)");
        telemetry.addData("Target cell", "%s%s", cellName(pose), flipTarget ? " (X override)" : "");
        if (activeCell != null) {
            telemetry.addData("Distance to cell", "%.1f in", pose.distance(activeCell));
        }
        telemetry.addData("Heading error", "%.1f deg", Math.toDegrees(headingErrorRad));
        telemetry.addData("Pose", formatPose(pose));
    }

    private void handleDriverButtons() {
        if (gamepad1.a && !prevA) {
            aiming = !aiming;
        }
        if (gamepad1.x && !prevX) {
            flipTarget = !flipTarget;
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            localizer.setPose(FieldLM1.audienceCornerPose(alliance()));
        }
        if (gamepad1.dpad_right && !prevDpadRight) {
            localizer.setPose(FieldLM1.farCornerPose(alliance()));
        }
        prevA = gamepad1.a;
        prevX = gamepad1.x;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
    }

    @Override
    protected double turnCommand(double forward, double strafe, double driverTurn) {
        Pose pose = localizer.pose();
        activeCell = chooseCell(pose);

        // Heading that points the robot's BACK at the cell: face the cell, then add 180 deg.
        double angleToCell = Math.atan2(activeCell.y() - pose.y(), activeCell.x() - pose.x());
        double desiredHeading = angleToCell + Math.PI;
        headingErrorRad = wrapAngle(desiredHeading - pose.heading());

        // Moving any stick hands control back to the driver.
        if (aiming && (Math.abs(forward) > STICK_CANCEL || Math.abs(strafe) > STICK_CANCEL
                || Math.abs(driverTurn) > STICK_CANCEL)) {
            aiming = false;
        }
        if (!aiming) {
            return driverTurn;
        }
        if (isLockedOn()) {
            return 0;
        }

        /*
         * Heading is counterclockwise-positive, but a positive turn command turns clockwise,
         * so a positive error (need to turn counterclockwise) needs a NEGATIVE turn command.
         */
        double turn = -AIM_KP * headingErrorRad;
        turn = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));
        if (Math.abs(turn) < AIM_MIN_TURN) {
            turn = Math.copySign(AIM_MIN_TURN, turn);
        }
        return turn;
    }

    // Audience half (y < 72) -> audience-side cell, far half -> far-side cell; X flips it.
    private Pose chooseCell(Pose pose) {
        boolean audienceHalf = pose.y() < FieldLM1.CENTER;
        if (flipTarget) {
            audienceHalf = !audienceHalf;
        }
        return audienceHalf ? FieldLM1.audienceCell(alliance()) : FieldLM1.farCell(alliance());
    }

    private String cellName(Pose pose) {
        boolean audienceHalf = pose.y() < FieldLM1.CENTER;
        if (flipTarget) {
            audienceHalf = !audienceHalf;
        }
        return audienceHalf ? "audience-side cell" : "far-side cell";
    }

    private boolean isLockedOn() {
        return Math.abs(Math.toDegrees(headingErrorRad)) <= AIM_TOLERANCE_DEG;
    }

    // Keep an angle between -180 and 180 degrees (in radians).
    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static String formatPose(Pose p) {
        return String.format("x %.1f, y %.1f, heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
    }
}
