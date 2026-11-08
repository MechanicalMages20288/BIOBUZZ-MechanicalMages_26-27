package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;
import org.firstinspires.ftc.teamcode.Java.Field.PoseStorage;
import org.firstinspires.ftc.teamcode.Java.Shooter.FlywheelSpeedModel;
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
 *
 * AUTO FLYWHEEL SPEED (gamepad 2): the flywheel speed is picked from the distance to the target
 * cell by FlywheelSpeedModel. It's on by default.
 *   Y            tap: auto speed ON / OFF (OFF = the fixed D-pad speed, like BasicTeleopLM1)
 *   D-pad        with auto ON, the D-pad adds a TRIM on top of the auto speed
 *                (right/left +25/-25, up +100, down clears the trim).
 */
public abstract class AimTeleopLM1 extends BasicTeleopLM1 {

    // Turning power per radian of heading error, and limits on the turning power.
    public static double AIM_KP = 1.2;
    public static double AIM_MAX_TURN = 0.6;
    public static double AIM_MIN_TURN = 0.06; // small push so it doesn't stall just short...
    public static double AIM_MIN_TURN_ABOVE_DEG = 6; // ...but only when this far off (no kick near target)
    public static double AIM_TOLERANCE_DEG = 2; // within this, it's aligned: stop and brake
    public static double AIM_TIMEOUT_SECONDS = 0.75; // give up turning after this long

    // Any stick pushed past this cancels auto-aim and gives control back to the driver.
    public static double STICK_CANCEL = 0.15;

    // Right-stick values smaller than this count as zero, so stick drift doesn't slowly turn the robot.
    public static double TURN_DEADBAND = 0.05;

    // Which alliance this teleop is for.
    protected abstract FieldLM1.Alliance alliance();

    protected PinpointLocalizer localizer;
    private GoBildaPinpointDriver pinpoint; // same Pinpoint, used for gyro recalibration + status
    private int pinpointProblemCount = 0;   // loops where the Pinpoint wasn't READY
    protected Pose startPose;
    protected String poseSource = "";

    private boolean aiming = false;
    private boolean aligned = false; // last tap of A finished lined up (shown on telemetry)
    private final ElapsedTime aimTimer = new ElapsedTime();
    private double aimStartErrorSign = 0; // which way the robot started turning (0 = not set yet)
    private boolean flipTarget = false;
    private Pose activeCell = null;
    private double headingErrorRad = 0;

    private boolean prevA, prevX, prevDpadLeft, prevDpadRight;

    // Auto flywheel speed from distance (gamepad 2 Y toggles it).
    private boolean autoSpeed = true;
    private boolean prevY2 = false;
    private double distanceToCell = 0;

    @Override
    public void init() {
        super.init();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        /*
         * Recalibrate the Pinpoint's gyro now, while the robot is sitting still at INIT.
         * A bad gyro calibration makes heading drift, which drags x/y off as you drive.
         * KEEP THE ROBOT STILL during INIT. The position is set again at START.
         */
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        pinpoint.recalibrateIMU();

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

    /*
     * Hook: called every loop right after the Pinpoint updates, before anything uses the pose.
     * The Limelight teleops override this to correct the pose from AprilTags.
     */
    protected void correctPose() {
    }

    // Hook: extra telemetry lines shown at the end of each loop.
    protected void extraTelemetry() {
    }

    @Override
    public void loop() {
        localizer.update();
        correctPose();
        handleDriverButtons();

        // Distance to the target cell, used for auto flywheel speed in flywheelTarget().
        Pose current = localizer.pose();
        distanceToCell = current.distance(chooseCell(current));

        // BasicTeleopLM1 drives and runs the shooter; it calls turnCommand() and
        // flywheelTarget() below.
        super.loop();

        Pose pose = localizer.pose();
        telemetry.addLine();
        if (autoSpeed) {
            int trim = launcherTargetVelocity - LAUNCHER_DEFAULT_VELOCITY;
            telemetry.addData("Flywheel speed", "AUTO %d + trim %d = %d ticks/s  (near/far x%.2f)%s",
                    FlywheelSpeedModel.velocityForDistance(distanceToCell), trim, flywheelTarget(),
                    FlywheelSpeedModel.nearFarMultiplier(distanceToCell),
                    FlywheelSpeedModel.tooClose(distanceToCell) ? "  (TOO CLOSE to score)" : "");
        } else {
            telemetry.addData("Flywheel speed", "MANUAL %d ticks/s (gamepad 2 Y for auto)", flywheelTarget());
        }
        telemetry.addData("Auto-aim", aiming ? "TURNING" : (aligned ? "ALIGNED (stopped)" : "off (tap A)"));
        telemetry.addData("Target cell", "%s%s", cellName(pose), flipTarget ? " (X override)" : "");
        if (activeCell != null) {
            telemetry.addData("Distance to cell", "%.1f in", pose.distance(activeCell));
        }
        telemetry.addData("Heading error", "%.1f deg", Math.toDegrees(headingErrorRad));
        telemetry.addData("Pose", formatPose(pose));

        // Pinpoint health: any non-READY moment (e.g. a bad read) can make the position jump.
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            pinpointProblemCount++;
        }
        telemetry.addData("Pinpoint", "%s   (problems seen: %d)", status, pinpointProblemCount);

        extraTelemetry();
    }

    private void handleDriverButtons() {
        if (gamepad1.a && !prevA) {
            aiming = !aiming;
            aligned = false;
            aimStartErrorSign = 0;
            aimTimer.reset();
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
        if (gamepad2.y && !prevY2) {
            autoSpeed = !autoSpeed;
        }
        prevY2 = gamepad2.y;

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
            aligned = false;
        }
        if (!aiming) {
            // Ignore tiny stick drift so the robot doesn't slowly turn on its own.
            return Math.abs(driverTurn) < TURN_DEADBAND ? 0 : driverTurn;
        }

        /*
         * ONE ALIGNMENT MOTION: one turn toward the target, then stop. It ends as soon as the
         * robot is lined up, OR has swung past the target (the error flipped sign), OR runs out
         * of time. It never turns back to fine-tune, so there's no hunting back and forth.
         * The drive motors are in BRAKE mode, so the robot stops and stays put.
         */
        if (aimStartErrorSign == 0) {
            aimStartErrorSign = Math.signum(headingErrorRad);
        }
        boolean crossedTarget = Math.signum(headingErrorRad) != aimStartErrorSign;
        if (isLockedOn() || crossedTarget || aimTimer.seconds() > AIM_TIMEOUT_SECONDS) {
            aiming = false;
            aligned = isLockedOn() || crossedTarget;
            return 0;
        }

        /*
         * Heading is counterclockwise-positive, but a positive turn command turns clockwise,
         * so a positive error (need to turn counterclockwise) needs a NEGATIVE turn command.
         */
        double turn = -AIM_KP * headingErrorRad;
        turn = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));
        // Minimum push only when far off; close to the target it eases in so it doesn't overshoot.
        if (Math.abs(Math.toDegrees(headingErrorRad)) > AIM_MIN_TURN_ABOVE_DEG && Math.abs(turn) < AIM_MIN_TURN) {
            turn = Math.copySign(AIM_MIN_TURN, turn);
        }
        return turn;
    }

    /*
     * Auto speed ON: physics-based speed for the current distance, plus the D-pad trim
     * (how far the D-pad has moved the manual target from its default). Auto OFF: manual speed.
     */
    @Override
    protected int flywheelTarget() {
        if (!autoSpeed) {
            return launcherTargetVelocity;
        }
        int trim = launcherTargetVelocity - LAUNCHER_DEFAULT_VELOCITY;
        int target = FlywheelSpeedModel.velocityForDistance(distanceToCell) + trim;
        return Math.max(0, Math.min(LAUNCHER_MAX_VELOCITY, target));
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
