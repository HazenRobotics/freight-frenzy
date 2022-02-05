package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.drives.TwoWheelTrackingLocalizerTippy;
import org.firstinspires.ftc.teamcode.localization.DistanceSensorLocalizer;
import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
//@Disabled
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RRMecanumDriveTippy42 drive = new RRMecanumDriveTippy42(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setLocalizer( new TrackingCameraLocalizer( hardwareMap, new Pose2d(  ) ) );
        //drive.setLocalizer( new DistanceSensorLocalizer( hardwareMap, new Vector2d( 4.5, 6.25 ), new Vector2d( 4.5, 6.25 ), new Vector2d( 7.25, 0 ), drive ) );
        //drive.setLocalizer( new TwoWheelTrackingLocalizerTippy( hardwareMap, drive ) );

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if(gamepad1.a) {
                drive.setPoseEstimate( new Pose2d(  ) );
            }
        }
        drive.stopCamera();
    }

}
