package org.firstinspires.ftc.teamcode.Tuning;

import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Pinpoint Check: reads the goBILDA Pinpoint directly through the FTC SDK, without Pedro.
 * Never drives any motors, so it's safe to run with the robot anywhere.
 *
 * Push the robot by hand and watch:
 *   - Status:     should say READY. Anything else tells you what's wrong (see the hint line).
 *   - Raw X / Y:  pod tick counts. They should change when you push the robot.
 *   - Position:   x / y in inches and heading in degrees.
 *
 * Gamepad 1 A: reset position and recalibrate the IMU (keep the robot still for a moment).
 */
@TeleOp(name = "Pinpoint Check", group = "Tuning")
public class PinpointCheck extends OpMode {

    // Must match the Pinpoint's name in the robot configuration.
    final String PINPOINT_NAME = "pinpoint";

    private GoBildaPinpointDriver pinpoint;
    private boolean prevA = false;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        /*
         * Use the SAME settings as the teleops and autos (pedro/Constants.java): pod type,
         * pod directions and pod offsets. Without the offsets, every turn would look like the
         * robot slid sideways. Raw tick counts don't depend on any of this.
         */
        PinpointConfig cfg = Constants.localizerConfig;
        if (cfg.podType.get() != null) {
            pinpoint.setEncoderResolution(cfg.podType.get());
        }
        pinpoint.setEncoderDirections(cfg.xPodDirection.get(), cfg.yPodDirection.get());
        pinpoint.setOffsets(cfg.xPodOffset.get(), cfg.yPodOffset.get(), DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint found. Press START, then push the robot by hand.");
    }

    @Override
    public void loop() {
        if (gamepad1.a && !prevA) {
            pinpoint.resetPosAndIMU();
        }
        prevA = gamepad1.a;

        pinpoint.update();

        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        telemetry.addData("Status", status);
        telemetry.addData("Hint", hint(status));
        telemetry.addLine();
        telemetry.addData("Raw X pod (ticks)", pinpoint.getEncoderX());
        telemetry.addData("Raw Y pod (ticks)", pinpoint.getEncoderY());
        telemetry.addLine();
        telemetry.addData("Position", "x %.2f in, y %.2f in, heading %.1f deg",
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("Device ID / version", "%d / %d", pinpoint.getDeviceID(), pinpoint.getDeviceVersion());
        telemetry.addData("Update rate", "%.0f Hz (loop %d us)", pinpoint.getFrequency(), pinpoint.getLoopTime());
        telemetry.addLine("Gamepad 1 A = reset position + IMU");
        telemetry.update();
    }

    private String hint(GoBildaPinpointDriver.DeviceStatus status) {
        switch (status) {
            case READY:
                return "OK. If raw X/Y don't change when pushed, the pods aren't turning.";
            case CALIBRATING:
                return "Calibrating IMU, keep the robot still for a second.";
            case NOT_READY:
                return "Pinpoint is starting up. If this stays, power-cycle the robot.";
            case FAULT_X_POD_NOT_DETECTED:
                return "X pod not detected: check its cable in the Pinpoint's X port.";
            case FAULT_Y_POD_NOT_DETECTED:
                return "Y pod not detected: check its cable in the Pinpoint's Y port.";
            case FAULT_NO_PODS_DETECTED:
                return "No pods detected: check both pod cables are in the Pinpoint X/Y ports.";
            case FAULT_IMU_RUNAWAY:
                return "IMU fault: keep the robot still and press A to recalibrate.";
            case FAULT_BAD_READ:
                return "Bad I2C read: check the I2C cable and that the config names the right port.";
            default:
                return "";
        }
    }
}
