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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Simple Mecanum Autonomous OpMode converted from StarterBotAuto.
 * Launches preload elements and drives away from the wall using encoder-based drive logic.
 */

@Autonomous(name = "Shoot and Leave Auto", group = "Autonomous")
public class ShootAndLeaveBasic extends OpMode {

    // Flywheel speeds (measured in encoder ticks per second)
    public final int LAUNCHER_TARGET_VELOCITY = 1250;
    public final int LAUNCHER_MIN_VELOCITY = 1200;

    // Drive parameters
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 402;

    double intakePower = 0;

    // Timers
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();

    // Drivetrain Motors (4 Mecanum Drive Motors)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Mechanism Hardware
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    private enum AutonomousState {
        LAUNCH,
        DRIVE,
        COMPLETE
    }

    private AutonomousState autonomousState;

    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;

        // Initialize 4 Mecanum drive motors matching hardware map names from TeleOp
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize mechanism hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        // Set motor directions matching TeleOp
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset drive encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Brake behavior
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Configure launcher PIDF and encoder mode
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        // Initial servo states and directions matching TeleOp
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
        switch (autonomousState) {
            case LAUNCH:
                launch(true);
                if (autoTimer.seconds() > 10) {
                    launch(false);
                    intakePower = 0;
                    autonomousState = AutonomousState.DRIVE;
                }
                break;

            case DRIVE:
                if (drive(DRIVE_SPEED, -1200, DistanceUnit.MM, 2)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                telemetry.addLine("Auto Complete!");
                break;
        }

        intake.setPower(intakePower);

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Motor Current Positions", "LF (%d), RF (%d), LB (%d), RB (%d)",
                leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        intake.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void launch(boolean input) {
        if (input) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        if (input && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(-1.0); // -1.0 feeds into launcher in correct direction
            intakePower = 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }

    /**
     * Drives all 4 Mecanum wheels forward/backward by a specified distance.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftFrontDrive.setTargetPosition((int) targetPosition);
        rightFrontDrive.setTargetPosition((int) targetPosition);
        leftBackDrive.setTargetPosition((int) targetPosition);
        rightBackDrive.setTargetPosition((int) targetPosition);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        if (Math.abs(targetPosition - leftFrontDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Rotates the robot in place using Mecanum drive.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        leftFrontDrive.setTargetPosition((int) leftTargetPosition);
        leftBackDrive.setTargetPosition((int) leftTargetPosition);
        rightFrontDrive.setTargetPosition((int) rightTargetPosition);
        rightBackDrive.setTargetPosition((int) rightTargetPosition);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        if (Math.abs(leftTargetPosition - leftFrontDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}
