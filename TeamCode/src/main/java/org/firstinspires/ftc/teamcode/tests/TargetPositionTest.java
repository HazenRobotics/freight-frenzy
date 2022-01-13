package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.localization.TrackingCameraLocalizer;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.TargetPositionCalculator;
import org.firstinspires.ftc.teamcode.vision.CameraTargetDistance;
import org.firstinspires.ftc.teamcode.vision.TensorFlowUtilBack;

@TeleOp(name = "TargetPositionTest", group = "Test")
public class TargetPositionTest extends OpMode {

	TensorFlowUtilBack tf;

	MecanumDrive mecanumDrive;
	RRMecanumDriveTippy42 drive;
	GamepadEvents gamepad;

	CameraTargetDistance cd;
	TargetPositionCalculator calculator;
	Vector2d movePosition = new Vector2d( 0, 0 );

	boolean driveToTarget = false, looping = true;

	static DcMotorSimple.Direction REV = DcMotorSimple.Direction.REVERSE;

	@Override
	public void init( ) {

		tf = new TensorFlowUtilBack( this );

		gamepad = new GamepadEvents( gamepad1 );
		mecanumDrive = new MecanumDrive( hardwareMap );
		mecanumDrive.setMotorDirections( REV, DcMotorSimple.Direction.FORWARD, REV, REV );
		drive = new RRMecanumDriveTippy42( hardwareMap );
		calculator = new TargetPositionCalculator( new Pose2d( -6,  -1, Math.toRadians( 180 )) );
		cd = new CameraTargetDistance( 720, 45, 90 - 22.5 );



		telemetry.addLine( "Init Finished!!" );

		drive.setCameraFrameOfReference( TrackingCameraLocalizer.CardinalDirection.EAST );
/*

		do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			drive.update( );
		} while( drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );
*/

		drive.setPoseEstimate( new Pose2d( 0, 0, Math.toRadians( 0 ) ) );

		telemetry.addLine( "Press 'a' to drive to most conf. target!!" );
		telemetry.update( );

		loopTargets( );
	}

	@Override
	public void loop( ) {

		if( !driveToTarget )
			mecanumDrive.drive( 0.7 * -gamepad.left_stick_y, gamepad.left_stick_x, 0.6 * gamepad.right_stick_x );

		if( gamepad.a.onPress( ) ) {

			if( !driveToTarget ) {

				drive.followTrajectorySequenceAsync( drive.trajectorySequenceBuilder( drive.getPoseEstimate( ) )
						.lineTo( movePosition )
//						.splineToLinearHeading( new Pose2d( moveDistance.getX( ), moveDistance.getY( ), Math.toRadians( 0 ) ), Math.toRadians( 0 ) )
						.build( ) );
				driveToTarget = true;

			} else {
				drive.cancelTrajectorySequence( );
				driveToTarget = false;
				// stop
			}
		}
		drive.update( );

		gamepad.update( );
		if( !drive.isBusy( ) && driveToTarget ) {
			driveToTarget = false;
		}
	}

	@Override
	public void stop( ) {
		looping = false;
		tf.stopTF( );
	}

	public void loopTargets( ) {

		new Thread( ( ) -> {
			tf.initTensorFlow( );
			tf.startTF( );

			while( looping ) {

				Recognition recognition = tf.identifyObject( );

				if( recognition != null ) {

					Pose2d position = drive.getPoseEstimate( );
					telemetry.addLine( "offset: " + position );

					movePosition = calculator.getTargetPosition( recognition, drive.getExternalHeading( ) );
					telemetry.addLine( "Corban: " + movePosition );
					movePosition = new Vector2d( movePosition.getX( ) + position.getX( ), movePosition.getY( ) + position.getY( ) );
					telemetry.addLine( "Corban + offset: " + movePosition );

//					Vector2d distance = cd.getDistance( recognition.getLeft( ), recognition.getWidth( ) );
//					distance = new Vector2d( -distance.getX( ), -distance.getY( ) );
//					telemetry.addLine( "Sam: " + distance );
//					distance = new Vector2d( distance.getX( ) + position.getX( ), distance.getY( ) + position.getY( ) );
//					telemetry.addLine( "Sam + offset: " + distance );

/*
					new Vector2d( movePosition.getX() * Math.cos(drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS )) + movePosition.getY() * Math.sin(drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS)),
							movePosition.getY() * Math.cos( drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS ) ) + movePosition.getY() * Math.sin(drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS )));
*/

					telemetry.update( );
				}

				try {
					Thread.sleep( 100 );
				} catch( InterruptedException ignored ) {
				}
			}

		} ).start( );
	}
}
