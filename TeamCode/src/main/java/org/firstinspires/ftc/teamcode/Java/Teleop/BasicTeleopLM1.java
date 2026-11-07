/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Java.Teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge. On top of a mecanum wheel drivetrain, it uses
 * one motor driving an intake roller, one motor driving transfer rollers that feed the launcher,
 * and a high-speed launcher motor.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
//@Disabled
public class BioBuzzStarterbotTeleopMecanum extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private DcMotor transfer = null;

    /*
     * GAMEPAD 2 (operator) CONTROLS
     *   A (hold)              intake: intake motor only
     *   B (hold)              outtake: intake AND transfer in reverse (also clears jams)
     *   Right trigger (hold)  FIRE: intake + transfer together, feeding balls into the flywheel
     *   D-pad right / left    flywheel speed +25 / -25 ticks/s
     *   D-pad up              flywheel speed +100 ticks/s
     *   D-pad down            reset flywheel speed to the default (1650)
     * The flywheel spins the whole time teleop is running.
     */

    // Intake motor power for A (intake) and B (outtake).
    public final double INTAKE_POWER = 1;
    public final double OUTTAKE_POWER = -1;

    /*
     * Power for the transfer rollers when firing. If the rollers push balls the wrong way,
     * flip this to -1.
     */
    public final double TRANSFER_FEED_POWER = 1;

    // Transfer power while outtaking with B (opposite of feeding).
    public final double TRANSFER_OUTTAKE_POWER = -TRANSFER_FEED_POWER;

    // Intake power while firing, so the intake also pushes balls up into the transfer.
    public final double INTAKE_FIRE_POWER = 1;

    // How far the right trigger must be pulled to fire.
    public final double FIRE_TRIGGER_THRESHOLD = 0.5;


    /*
     * Launcher speeds, all in encoder ticks per second. The encoder counts 28 ticks per motor
     * revolution, so RPM = ticks/s * 60 / 28 (1650 ticks/s is about 3540 RPM).
     *
     * LAUNCHER_MAX_VELOCITY is about the fastest a goBILDA 6000 RPM (1:1) motor can spin with
     * no load (2800 ticks/s = 6000 RPM). With the flywheel on it tops out a bit below that.
     */
    final int LAUNCHER_MAX_VELOCITY = 2800;
    public final int LAUNCHER_DEFAULT_VELOCITY = 1650; // tuned with FlywheelPIDFTuner
    public int launcherTargetVelocity = LAUNCHER_DEFAULT_VELOCITY;
    public final int LAUNCHER_MIN_VELOCITY = 1000;

    // How close (in ticks/s) the launcher must be to the target to count as "at speed".
    public final int LAUNCHER_VELOCITY_TOLERANCE = 50;

    // Each tap of d-pad right/left changes the target by the small step; up adds the big step.
    public final int LAUNCHER_VELOCITY_STEP = 25;
    public final int LAUNCHER_VELOCITY_BIG_STEP = 100;

    // Last loop's d-pad state, so one tap changes the target once.
    boolean dpadRightWasDown = false;
    boolean dpadLeftWasDown = false;
    boolean dpadUpWasDown = false;
    boolean dpadDownWasDown = false;

    // Whether the launcher is at its target velocity, updated every loop in launch().
    boolean launcherAtSpeed = false;


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        // Reverse in code (not by swapping wires) so the motor and its encoder stay matched.
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        // Reversed so positive power feeds balls toward the flywheel.
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        transfer.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40,  0, 0, 14.4));

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested
         * by the driver. Moving the left joystick forwards/back moves all motors forwards/back,
         * moving the right joystick left/right rotates the robot clockwise or counterclockwise,
         * and moving the left joystick left moves the motors in the right way to create a sideways
         * "strafe" movement. Combinations of these inputs can be used to create more complex maneuvers.
         * Note, moving the joystick forward on most gamepads results in a negative signal, so
         * we invert it before passing it to the function.
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * Intake: hold A to intake, B to outtake. The transfer is handled in launch(): it runs
         * forward when firing (right trigger) and in reverse with B.
         */
        if (gamepad2.a) {
            intakePower = INTAKE_POWER;
        } else if (gamepad2.b) {
            intakePower = OUTTAKE_POWER;
        } else {
            intakePower = 0;
        }

        // D-pad adjusts the flywheel target: right/left small steps, up big step, down resets.
        if (gamepad2.dpad_right && !dpadRightWasDown) {
            changeLauncherTarget(LAUNCHER_VELOCITY_STEP);
        }
        if (gamepad2.dpad_left && !dpadLeftWasDown) {
            changeLauncherTarget(-LAUNCHER_VELOCITY_STEP);
        }
        if (gamepad2.dpad_up && !dpadUpWasDown) {
            changeLauncherTarget(LAUNCHER_VELOCITY_BIG_STEP);
        }
        if (gamepad2.dpad_down && !dpadDownWasDown) {
            launcherTargetVelocity = LAUNCHER_DEFAULT_VELOCITY;
        }
        dpadRightWasDown = gamepad2.dpad_right;
        dpadLeftWasDown = gamepad2.dpad_left;
        dpadUpWasDown = gamepad2.dpad_up;
        dpadDownWasDown = gamepad2.dpad_down;

        launch();

        /*
         * Launcher status, shown first so it's at the top of the Driver Station screen.
         * launcherAtSpeed was just updated inside launch(), so it reflects this loop.
         */
        telemetry.addData("Shooter", launcherAtSpeed ? "READY TO FIRE" : "NOT READY");
        telemetry.addData("Target velocity", "%d ticks/s", launcherTargetVelocity);
        telemetry.addData("Actual velocity", "%.0f ticks/s", launcher.getVelocity());

        intake.setPower(intakePower);

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        transfer.setPower(0);
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward - strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward + strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launch() {
        // Keep the launcher spinning at the target velocity the whole time teleop is running.
        launcher.setVelocity(launcherTargetVelocity);

        // Check whether the launcher is at its target velocity (shown on telemetry as ready/not ready).
        launcherAtSpeed = Math.abs(launcher.getVelocity() - launcherTargetVelocity) <= LAUNCHER_VELOCITY_TOLERANCE;

        /*
         * FIRE: hold gamepad 2 right trigger to run every stage (intake + transfer) and push balls
         * into the flywheel. This overrides A/B for the intake while held.
         */
        if (gamepad2.right_trigger > FIRE_TRIGGER_THRESHOLD) {
            transfer.setPower(TRANSFER_FEED_POWER);
            intakePower = INTAKE_FIRE_POWER;
        } else if (gamepad2.b) {
            // Outtake: run the transfer backward too, pushing balls back out (clears jams).
            transfer.setPower(TRANSFER_OUTTAKE_POWER);
        } else {
            transfer.setPower(0);
        }
    }

    // Change the flywheel target, kept between 0 and the max.
    void changeLauncherTarget(int amount) {
        launcherTargetVelocity = Math.max(0, Math.min(launcherTargetVelocity + amount, LAUNCHER_MAX_VELOCITY));
    }
}