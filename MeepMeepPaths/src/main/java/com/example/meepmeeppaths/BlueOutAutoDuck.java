package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueOutAutoDuck implements MeepMeepPath {

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( -25.875, 62.1875, Math.toRadians( 270 ) ) )

				.splineToLinearHeading( MeepMeepPath.getHubPosition( 22.5, 270, 1 /*shippingHubHeightToInches( height )*/, true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
//											robot.dumpBucket( );
//											robot.lift.setDefaultHeightVel( 1000 );
				} )
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

				// move to barrier to park
				.turn( Math.toRadians(90) )
				.setTangent( Math.toRadians( 180 ) )
				.splineToConstantHeading( new Vector2d( -36, 24 ), Math.toRadians( 270 ) )
				.splineToConstantHeading( new Vector2d( -12, 0 ), Math.toRadians( 0 ) )
				.splineToConstantHeading( new Vector2d( 12, 24 ), Math.toRadians( 90 ) )
				.lineToConstantHeading( new Vector2d( 12, 44 ) )

				/*.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( 11.5, 44, 0 ), Math.toRadians( -45 ) )
				.lineToLinearHeading( new Pose2d( 62, 44, 0 ) )*/

				.waitSeconds( 6 )

				.build( );
	}
}