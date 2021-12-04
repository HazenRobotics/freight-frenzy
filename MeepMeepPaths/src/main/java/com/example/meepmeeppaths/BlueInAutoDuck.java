package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueInAutoDuck implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( -6.375, 62.1875, Math.toRadians( 270 ) ) )
				.splineToLinearHeading( MeepMeepPath.getHubPosition( -22.5, 270, 1 /*shippingHubHeightToInches( height )*/, true ), Math.toRadians( 270 - 22.5 ) )

				.waitSeconds( 1.2 )

				// Duck spin
				.lineToLinearHeading( new Pose2d( -58.5, 56, Math.toRadians( -90 ) ) )
				.addTemporalMarker( ( ) -> {
//											robot.spinner.setPower( 0.5 );
				} )
				.waitSeconds( 3.2 )
				.addTemporalMarker( ( ) -> {
//											robot.spinner.setPower( 0 );
				} )

				// pickup the duck
				.addTemporalMarker( ( ) -> {
//											robot.intake.setPower( 0.6 );
				} )
				.setTangent( 0 )
				.splineToLinearHeading( new Pose2d( -50, 62.1875, Math.toRadians( -90 ) ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( -13 - 5, 62.1875 ) )
				.addTemporalMarker( ( ) -> {
//											robot.intake.setPower( 0 );
				} )

				// drop duck in bottom (always)
				.splineToLinearHeading( MeepMeepPath.getHubPosition( 0, 270, 1 + 4/*shippingHubHeightToInches( ShippingHubHeight.LOW )*/, true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
//											robot.dumpBucket( );
//											robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, 63.5, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 48, 63.5 ) )
				.lineToConstantHeading( new Vector2d( 18, 63.5 ) )
				.splineToSplineHeading( MeepMeepPath.getHubPosition( -22.5, 270, 1 /*shippingHubHeightToInches( height )*/, true ), Math.toRadians( 270 ) )
				.waitSeconds( 1.2 )

				// move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 11.5, 44, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.lineToConstantHeading( new Vector2d( 62, 44 ) )
				.build();
	}
}
