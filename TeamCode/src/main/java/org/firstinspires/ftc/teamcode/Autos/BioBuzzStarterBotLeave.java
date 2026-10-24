package org.firstinspires.ftc.teamcode.Autos;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autonomous for the goBILDA StarterBot with MECANUM wheels (BIOBUZZ 2026-2027).
 *
 * Park path (Leave only):
 *   1. STRAIGHT:    drive straight at -0.9 power for 2.3 seconds.
 *   2. STRAFE_LEFT: strafe left at 0.4 power for 0.5 seconds.
 */

@Autonomous(name = "Mec BioBuzz StarterBot Leave Auto", group = "StarterBot")
//@Disabled
public class BioBuzzStarterBotLeave extends OpMode {

    /*
     * Park path parameters:
     *   1. STRAIGHT:    drive straight at -0.9 power for 2.3 seconds.
     *   2. STRAFE_LEFT: strafe left at 0.4 power for 0.5 seconds.
     */
    final double STRAIGHT_POWER = -0.9;
    final double STRAIGHT_SECONDS = 1.4;
    final double STRAFE_LEFT_POWER = 0.4;
    final double STRAFE_LEFT_SECONDS = 0.5;

    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // Whether the current timed move has started (and its timer been reset) yet.
    private boolean moveStarted = false;

    private enum AutonomousState {
        STRAIGHT,
        STRAFE_LEFT,
        COMPLETE
    }

    private AutonomousState autonomousState;

    @Override
    public void init() {
        autonomousState = AutonomousState.STRAIGHT;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Same directions as the mecanum teleop.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

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
        switch (autonomousState) {
            case STRAIGHT:
                if (drive(STRAIGHT_POWER, STRAIGHT_SECONDS)) {
                    autonomousState = AutonomousState.STRAFE_LEFT;
                }
                break;
            case STRAFE_LEFT:
                if (strafe(STRAFE_LEFT_POWER, STRAFE_LEFT_SECONDS)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
            case COMPLETE:
                stopDrive();
                telemetry.addLine("Auto Complete!");
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Auto time", "%.1f s", autoTimer.seconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
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