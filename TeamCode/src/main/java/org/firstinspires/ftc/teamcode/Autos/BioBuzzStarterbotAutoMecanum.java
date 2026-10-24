/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autos;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autonomous for the goBILDA StarterBot with MECANUM wheels (BIOBUZZ 2026-2027).
 * Adapted from the tank-drive StarterBotAuto.
 *
 * The robot starts against the alliance wall facing the hive, launches its projectiles, then
 * parks in its alliance's loading zone (see the "Park path" notes below).
 * Hardware names and motor directions match BioBuzzStarterbotTeleopMecanum.
 *
 * The drive motors have no encoders, so every move is TIMED: the wheels run at a set power
 * for a set number of seconds. Distances change with battery level and floor grip, so tune
 * the seconds on the real field.
 */

@Autonomous(name = "Mec BioBuzz StarterBot Auto", group = "StarterBot")
//@Disabled
public class BioBuzzStarterbotAutoMecanum extends OpMode {

    /*
     * Launcher speeds in encoder ticks per second (same units as the teleop).
     * 1250 ticks/s is about 2680 RPM.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    /*
     * Start feeding once the launcher is at speed, OR after this many seconds of spin-up,
     * whichever comes first. The time fallback means the auto still shoots if the launcher
     * encoder isn't reading.
     */
    final double SPIN_UP_SECONDS = 2;

    /*
     * Windmill power used to feed the launcher. Your teleop feeds with -1 (left bumper),
     * so we use the same here. Flip to 1 if the windmill spins the wrong way.
     */
    final double WINDMILL_FEED_POWER = -1;

    // How long (seconds) to spend launching before driving away.
    final double LAUNCH_SECONDS = 10;

    /*
     * Park path. Start: back against your alliance wall, on the tile level with the hive cell
     * FARTHER from your loading zone, facing the hive. The loading zone is then about 2 tiles
     * (~48 in) to the robot's LEFT along the same wall, for both red and blue (the field is
     * mirrored), so the same path works for either alliance.
     *
     *   1. LEAVE_WALL:   drive forward a few inches so the robot doesn't scrape the wall.
     *   2. STRAFE_LEFT:  strafe left ~48 in, to line up with the loading zone.
     *   3. BACK_TO_WALL: drive backward into the loading zone until the back hits the wall.
     *
     * These are timed guesses, assuming 312 RPM drive motors and a full battery. Tune them on
     * the field: STRAFE_LEFT_SECONDS matters most. BACK_TO_WALL can run a little long, since
     * the wall just stops the robot.
     */
    final double LEAVE_WALL_POWER = 0.3;
    final double LEAVE_WALL_SECONDS = 0.25;
    final double STRAFE_LEFT_POWER = -0.4; // negative = left
    final double STRAFE_LEFT_SECONDS = 2.4;
    final double BACK_TO_WALL_POWER = -0.3;
    final double BACK_TO_WALL_SECONDS = 0.6;

    double intakePower = 0;

    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    // Whether the current timed move has started (and its timer been reset) yet.
    private boolean moveStarted = false;

    private enum AutonomousState {
        LAUNCH,
        LEAVE_WALL,
        STRAFE_LEFT,
        BACK_TO_WALL,
        COMPLETE
    }

    private AutonomousState autonomousState;

    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        // Same directions as the mecanum teleop.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * No drive encoders, so run the wheels on plain power. (RUN_USING_ENCODER without an
         * encoder plugged in would make the motors run at full speed.)
         */
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        autoTimer.reset();
    }

    @Override
    public void loop() {
        intakePower = 0;

        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                if (autoTimer.seconds() > LAUNCH_SECONDS) {
                    launch(false);
                    intakePower = 0;
                    autonomousState = AutonomousState.LEAVE_WALL;
                }
                break;
            case LEAVE_WALL:
                if (drive(LEAVE_WALL_POWER, LEAVE_WALL_SECONDS)) {
                    autonomousState = AutonomousState.STRAFE_LEFT;
                }
                break;
            case STRAFE_LEFT:
                if (strafe(STRAFE_LEFT_POWER, STRAFE_LEFT_SECONDS)) {
                    autonomousState = AutonomousState.BACK_TO_WALL;
                }
                break;
            case BACK_TO_WALL:
                if (drive(BACK_TO_WALL_POWER, BACK_TO_WALL_SECONDS)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            case COMPLETE:
                stopDrive();
                telemetry.addLine("Auto Complete!");
                break;
        }

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Launcher velocity", "target %d, actual %.0f ticks/s",
                LAUNCHER_TARGET_VELOCITY, launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
        launcher.setVelocity(0);
        windmillServo.setPower(0);
    }

    void launch(boolean input) {
        if (input) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        boolean readyToFeed = launcher.getVelocity() > LAUNCHER_MIN_VELOCITY
                || autoTimer.seconds() > SPIN_UP_SECONDS;

        if (input && readyToFeed) {
            windmillServo.setPower(WINDMILL_FEED_POWER);
            intakePower = 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }

    /**
     * Drive straight. Positive power = forward, negative = backward.
     * @return true once the move has run for the given seconds (motors are then stopped).
     */
    boolean drive(double power, double seconds) {
        return timedMove(power, power, power, power, seconds);
    }

    /**
     * Strafe sideways. Positive power = right, negative = left.
     * Matches the teleop: LF +, RF -, LB -, RB + strafes right.
     */
    boolean strafe(double power, double seconds) {
        return timedMove(power, -power, -power, power, seconds);
    }

    /**
     * Rotate in place. Positive power = counterclockwise (left), negative = clockwise.
     */
    boolean rotate(double power, double seconds) {
        return timedMove(-power, power, -power, power, seconds);
    }

    /*
     * Runs all four wheels at the given powers. On the first call of a move it starts the
     * timer; once the time is up it stops the wheels, readies the next move, and returns true.
     */
    boolean timedMove(double lf, double rf, double lb, double rb, double seconds) {
        if (!moveStarted) {
            moveTimer.reset();
            moveStarted = true;
        }

        if (moveTimer.seconds() < seconds) {
            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);
            return false;
        }

        stopDrive();
        moveStarted = false;
        return true;
    }

    void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
