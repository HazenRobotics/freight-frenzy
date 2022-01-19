package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.vision.CameraTargetDistance;
import org.firstinspires.ftc.teamcode.vision.TensorFlowUtilBack;

@TeleOp(name = "TargetObjectPositionTest", group = "Test")
@Disabled
public class TargetObjectPositionTest extends OpMode {

	TensorFlowUtilBack tf;

	MecanumDrive mecanumDrive;

	RRMecanumDriveTippy42 drive;

	CameraTargetDistance cd;

	boolean driveToTarget = false;

	GamepadEvents gamepad;

	boolean looping = true;

	Vector2d moveDistance = new Vector2d( 0, 0 );

	static DcMotorSimple.Direction REV = DcMotorSimple.Direction.REVERSE;

	@Override
	public void init( ) {

		tf = new TensorFlowUtilBack( this );
		gamepad = new GamepadEvents( gamepad1 );
		mecanumDrive = new MecanumDrive( hardwareMap );
		mecanumDrive.setMotorDirections( REV, DcMotorSimple.Direction.FORWARD, REV, REV );
		drive = new RRMecanumDriveTippy42( hardwareMap );
		cd = new CameraTargetDistance( 720, 45, 90 - 22.5 );

		tf.initTensorFlow( );
		tf.startTF( );

		telemetry.addLine( "Init Finished!!" );

		//drive.setCameraFrameOfReference( TrackingCameraLocalizer.CardinalDirection.SOUTH );

		/*do {
			telemetry.addLine( "Getting pose estimate. Please wait..." );
			telemetry.update( );
			drive.update( );
		} while( drive.getPoseConfidence( ).compareTo( T265Camera.PoseConfidence.Medium ) < 0 );*/

		telemetry.addLine( "Press 'a' to drive to most conf. target!!" );
		telemetry.update( );

		loopTargets( );
	}

	@Override
	public void loop( ) {

		if( !driveToTarget )
			mecanumDrive.drive( -gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x );

		if( gamepad.a.onPress( ) ) {

			if( !driveToTarget ) {

				drive.setPoseEstimate( new Pose2d( 0, 0, Math.toRadians( 0 ) ) );

				drive.followTrajectorySequenceAsync( drive.trajectorySequenceBuilder( drive.getPoseEstimate( ) )
						.lineTo( moveDistance )
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

			while( looping ) {

				Recognition recognition = tf.identifyObject( );

				if( recognition != null ) {

					double left = recognition.getLeft( );
					double width = recognition.getWidth( );

					moveDistance = cd.getDistance( left, width );
					telemetry.addLine( "Move Distance (before): " + moveDistance );
					moveDistance = new Vector2d( -moveDistance.getX( ), -moveDistance.getY( ) );
					telemetry.addLine( "Move Distance: " + moveDistance );

					/*new Vector2d( moveDistance.getX() * Math.cos(drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS )) + moveDistance.getY() * Math.sin(drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS)),
							moveDistance.getY() * Math.cos( drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS ) ) + moveDistance.getY() * Math.sin(drive.getRawExternalHeading() + recognition.estimateAngleToObject( AngleUnit.RADIANS )));
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
