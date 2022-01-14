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
		return drive.trajectorySequenceBuilder( new Pose2d( -6.375, 62.1875, Math.toRadians( 270 ) ) )

				// move to dump initial block in designated layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( height );
				} )
				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( MeepMeepPath.getHubPosition( -22.5, 270, 4, true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )

				.waitSeconds( 0.8 )

				// move to grab block 1
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.lineToConstantHeading( new Vector2d( 48, wallPos ) )
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 1 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 2
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.lineToConstantHeading( new Vector2d( 50, wallPos ) )
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 2 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200, () -> robot.bucket.setAngle( RRTippyBot.BUCKET_ANGLE_INTAKE ) );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 3
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.intakeBlocks( 0.6, 1, 500 ); // should stop the intake after 1 block has been intaken
				} )
				.lineToConstantHeading( new Vector2d( 52, wallPos ) )
				.lineToConstantHeading( new Vector2d( 18, wallPos ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0 );
				} )

				// move to dump block 3 in the top layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.HIGH );
				} )
				.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 7, true ), Math.toRadians( 270 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
				} )
				.addTemporalMarker( ( ) -> {
//					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
//					robot.odometryLift.raise( );
//					robot.lift.setHeightVelocity( 1200, 23 );
				} )
				.waitSeconds( 0.8 )

				// turn towards the
				.turn( Math.toRadians( 110 ) )
				/*// move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 11.5, 44, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 62, 44 ) )*/
				.build( );
	}
}
