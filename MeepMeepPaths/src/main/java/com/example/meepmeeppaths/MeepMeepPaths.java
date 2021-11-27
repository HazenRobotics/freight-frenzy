package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;

public class MeepMeepPaths {

	// 13" robot (l, w): 13.25, 13.5
	// 18" robot (l, w): 17.375, 17.5
	static double robotLength = 17.375;
	static double robotWidth = 17.5; // with belts

	static final double tileSize = 23;
	static final double tileConnector = 0.75;

	static final double hubRadius = 9;

	public static void main( String[] args ) {

		MeepMeep mm = new MeepMeep( 750 )
				.setBackground( MeepMeep.Background.FIELD_FREIGHT_FRENZY )
				.setTheme( new ColorSchemeBlueDark( ) )
				.setBackgroundAlpha( 1f )
				.setBotDimensions( robotWidth, robotLength )
				.setConstraints( 30, 30, Math.toRadians( 60 ), Math.toRadians( 60 ), 17 )
				.followTrajectorySequence( drive ->
								drive.trajectorySequenceBuilder( new Pose2d( -25.875, 62.1875, Math.toRadians( 270 ) ) )

										.splineToLinearHeading( getHubPosition( 22.5, 270, 1 /*shippingHubHeightToInches( height )*/, true ), Math.toRadians( 270 - 22.5 ) )
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
										.splineToLinearHeading( getHubPosition( 0, 270, 1 + 4/*shippingHubHeightToInches( ShippingHubHeight.LOW )*/, true ), Math.toRadians( 270 - 22.5 ) )
										.addTemporalMarker( ( ) -> {
//											robot.dumpBucket( );
//											robot.lift.setDefaultHeightVel( 1000 );
										} )
										.waitSeconds( 1.2 )

										// move to barrier to park
										.setTangent( Math.toRadians( 90 ) )
										.splineToLinearHeading( new Pose2d( 11.5, 40.25, 0 ), Math.toRadians( -45 ) )
										.lineToLinearHeading( new Pose2d( 62, 40.25, 0 ) )

										.waitSeconds( 6 )

										.build( )
				).start( );
	}


	/*

	13" blue out
	drive.trajectorySequenceBuilder( new Pose2d( -30.125, 64.25, Math.toRadians( 270 ) ) )
								.lineToSplineHeading( getHubPosition( 22.5, 270, 1, true ) )




	 */

	/**
	 * @param angle           the number of degrees to turn to reach the side of the shipping hub
	 * @param angleOffset     the starting angle of the robot
	 * @param distanceFromHub the distance away from the shipping hub base to be
	 * @param blueSide        whether or not the robot is on the blue side
	 * @return the position (Pose2D) of where to go
	 */
	public static Pose2d getHubPosition( double angle, double angleOffset, double distanceFromHub, boolean blueSide ) {
		double x = tileConnector / 2 + tileSize / 2 + Math.sin( Math.toRadians( angle ) ) * (hubRadius + distanceFromHub + robotLength / 2);
		double y = tileConnector + tileSize + Math.cos( Math.toRadians( angle ) ) * (hubRadius + distanceFromHub + robotLength / 2);
		return new Pose2d( -x, y * (blueSide ? 1 : -1), Math.toRadians( angleOffset + angle ) );
	}
}