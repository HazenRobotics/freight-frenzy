package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueInAutoDuck implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		double wallPos = 64.625;
		return drive.trajectorySequenceBuilder( new Pose2d( -42.5, -64.125, Math.toRadians( 90 ) ) )

				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.4 );
//					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
				} )
				.setTangent( Math.toRadians( 260 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( new Pose2d( -40, -60, Math.toRadians( 90 ) ), Math.toRadians( 90 ) )
				.strafeRight( 2 )
				.strafeLeft( 4 )

				//.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
//					robot.sleepRobot( 1.5 );
//					robot.intake.setPower( 0 );
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
//					robot.stopDuckScanning( );
//					robot.stopTF( );
				} )
				//.waitSeconds( 1.0 )

				// drop duck in top
				.setTangent( Math.toRadians( 40 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( MeepMeepPath.getHubPosition( -45, 90, 9, false ), Math.toRadians( 90 ) )
				.waitSeconds( 0.5 )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.sleepRobot( 1 );
//					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.5 )
				//Line up for parking
				.lineToLinearHeading( new Pose2d( -36, -44, Math.toRadians( 0 ) ) )
				/*


				// move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( 11.5, 44, 0 ), Math.toRadians( -45 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, 44, 0 ) )
				*/
				/*.setVelConstraint( new MecanumVelocityConstraint( 45, 11.5 ) )
				.setTangent( Math.toRadians( 90 ) )
				.splineToConstantHeading( new Vector2d( -55, 36 ), Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( -12, 0, Math.toRadians( 0 ) ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 12, 44 ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> {

				} )
				.waitSeconds( 1.2 )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, 44, 0 ) )*/
				.build( );
	}
}
