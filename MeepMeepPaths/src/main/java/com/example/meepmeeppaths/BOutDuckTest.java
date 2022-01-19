package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BOutDuckTest implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( -25.875, 62.1875, Math.toRadians( 270 ) ) )

				// Duck spin
				.setTangent( Math.toRadians( 225 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( new Pose2d( -61, 57, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> {
//					robot.spinner.setVelocity( 325 );
				} )
				.waitSeconds( 1.75 )
				.addTemporalMarker( ( ) -> {
//					robot.spinner.setPower( 0 );
				} )

				// start duck scanning, move lift up, and move to drop off block
				.addTemporalMarker( ( ) -> {
//					robot.startDuckScanning( 250 );
//					robot.liftToShippingHubHeight( height );
				} )

				.setTangent( Math.toRadians( 300 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( MeepMeepPath.getHubPosition( 45, 270, 5, true ), Math.toRadians( 230 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
				} )
				.waitSeconds( 1.2 )
				// pickup the duck
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.4 );
//					robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE );
				} )
				.setTangent( Math.toRadians( 100 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( new Pose2d( -40, 60, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
//					robot.stopDuckScanning( );
//					robot.stopTF( );
				} )

				// drop duck in top
				.setTangent( Math.toRadians( 320 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( MeepMeepPath.getHubPosition( 45, 270, 7, true ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1000 );
				} )
				/*.addTemporalMarker( ( ) -> {
					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
					robot.odometryLift.raise( );
				} )*/
				.waitSeconds( 1.2 )
				.splineToLinearHeading( new Pose2d( -36, 44, Math.toRadians( 0 ) ), Math.toRadians( 180 ) )

				// move to barrier to park
				/*.setTangent( Math.toRadians( 90 ) )
				.splineToConstantHeading( new Vector2d( -55, 36 ), Math.toRadians( 270 ) )
				.splineToSplineHeading( new Pose2d( -12, 0, Math.toRadians( 0 ) ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 12, 44 ), Math.toRadians( 90 ) )
//				.splineToLinearHeading( new Pose2d( 11.5, 44, 0 ), Math.toRadians( -45 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, 44, 0 ) )*/
				.build( );
//		robot.drive.followTrajectorySequence( afterPickupDuck );
	}

}