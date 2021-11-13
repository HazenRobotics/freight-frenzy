package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveHex42;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

//@Autonomous
public class AutoLocalizationMap extends LinearOpMode {

    RRMecanumDriveHex42 drive;

    final String MAP_NAME = "autoLocalizationTest";

    @Override
    public void runOpMode() throws InterruptedException {


        drive = new RRMecanumDriveHex42(hardwareMap);

        telemetry.addLine("Make sure the robot is in the center of the field. Press start to begin auto mapping.");
        telemetry.update();

        waitForStart();

        drive.update();
        while (drive.getPoseConfidence().compareTo(T265Camera.PoseConfidence.Medium) < 0) {
            telemetry.addLine("Waiting for camera to recognize position...");
            telemetry.update();
            drive.update();
        }

        Localizer localizer = drive.getLocalizer();

        drive.setLocalizer(new MecanumDrive.MecanumLocalizer( drive ));
        //move around to make sure camera gets a good lock
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).forward(4).build());
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).back(8).build());
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).forward(4).build());
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeLeft(4).build());
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeRight(8).build());
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).strafeLeft(4).build());
        drive.followTrajectorySequence( drive.trajectorySequenceBuilder( drive.getPoseEstimate() ).lineToLinearHeading( new Pose2d( 0, 0, 0 ) ).build() );

        //set localizer back to camera
        drive.setLocalizer(localizer);
        drive.setPoseEstimate( new Pose2d( 0, 0 ) );

        telemetry.addLine("Mapping!");
        telemetry.update();

        TrajectorySequence redSide =
                drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo(new Vector2d(10, 0))
                .lineTo(new Vector2d(10, -60))
                .lineTo(new Vector2d(-12, -60))
                .lineTo(new Vector2d(-12, -46))
                .lineTo(new Vector2d(-32, -46))
                .lineTo(new Vector2d(-36, 0))
                .lineTo(new Vector2d(-36, -60))
                .lineTo(new Vector2d(-52, -60))
                .lineTo(new Vector2d(-52, 0))
                .lineTo(new Vector2d(0, 0))
                .build();
        drive.followTrajectorySequence(redSide);

        TrajectorySequence blueSide =
                drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo(new Vector2d(10, 0))
                .lineTo(new Vector2d(10, 60))
                .lineTo(new Vector2d(-12, 60))
                .lineTo(new Vector2d(-12, 46))
                .lineTo(new Vector2d(-32, 46))
                .lineTo(new Vector2d(-36, 0))
                .lineTo(new Vector2d(-36, 60))
                .lineTo(new Vector2d(-52, 60))
                .lineTo(new Vector2d(-52, 0))
                .lineTo(new Vector2d(0, 0))
                .build();
        drive.followTrajectorySequence(blueSide);

        TrajectorySequence redSideDepot = drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo( new Vector2d( 10, 0 ) )
                .lineTo(new Vector2d(10, -42))
                .setVelConstraint(new MecanumVelocityConstraint(50, 17))
                .lineTo(new Vector2d(60,-42))
                .setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo(new Vector2d(60,-36))
                .lineTo(new Vector2d(60,-60))
                .lineTo(new Vector2d(36,-60))
                .lineTo(new Vector2d(36,-42))
                .setVelConstraint(new MecanumVelocityConstraint(50, 17))
                .lineTo(new Vector2d(8, -42))
                .setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo( new Vector2d( 10, 0 ) )
                .lineTo(new Vector2d(0, 0))
                .build();
        drive.followTrajectorySequence(redSideDepot);


        TrajectorySequence blueSideDepot = drive.trajectorySequenceBuilder(drive.getPoseEstimate()).setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo( new Vector2d( 10, 0 ) )
                .lineTo(new Vector2d(10, 42))
                .setVelConstraint(new MecanumVelocityConstraint(50, 17))
                .lineTo(new Vector2d(60,42))
                .setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo(new Vector2d(60,36))
                .lineTo(new Vector2d(60,60))
                .lineTo(new Vector2d(36,60))
                .lineTo(new Vector2d(36,42))
                .setVelConstraint(new MecanumVelocityConstraint(50, 17))
                .lineTo(new Vector2d(8, 42))
                .setVelConstraint(new MecanumVelocityConstraint(15, 17))
                .lineTo( new Vector2d( 10, 0 ) )
                .lineTo(new Vector2d(0, 0))
                .build();
        drive.followTrajectorySequence(blueSideDepot);

        telemetry.addLine("Exporting map...");
        telemetry.update();
        drive.exportLocalizationMap(MAP_NAME);
        telemetry.addLine("Finished!");
        telemetry.update();
        requestOpModeStop();
    }
}
