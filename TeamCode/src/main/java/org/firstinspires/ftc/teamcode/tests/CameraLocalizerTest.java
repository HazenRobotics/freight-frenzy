package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;

@TeleOp(name = "Camera Test", group = "test")
public class CameraLocalizerTest extends OpMode {
    TrackingCameraLocalizer localizer;

    @Override
    public void init() {
        localizer = new TrackingCameraLocalizer();

    }

    @Override
    public void loop() {
        localizer.update();
        telemetry.addData("Current Position",localizer.poseEstimate);
        telemetry.addData("Current Velocity",localizer.poseVelocity);
        telemetry.addData("Current Acceleration",localizer.poseAcceleration);
        telemetry.update();

    }
}
