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
				.setTangent( Math.toRadians( 225 ) ) // 135 on red
				.splineToLinearHeading( new Pose2d( -62, 58, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )
				.addTemporalMarker( ( ) -> {
//					robot.spinner.setPower( 0.6 );
				} )
				.waitSeconds( 3.0 )
				.addTemporalMarker( ( ) -> {
//					robot.spinner.setPower( 0 );
				} )

				// TODO: start duck scanning
				.addTemporalMarker( ( ) -> {
//					robot.startDuckScanning( 250 );
				} )

				// move lift up and go to drop off block
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( height );

				} )

				.setTangent( Math.toRadians( 300 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( MeepMeepPath.getHubPosition( 22.5, 270,1 /*robot.shippingHubDistance( height )*/, true ), Math.toRadians( 270 + 22.5 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.waitSeconds( 1.2 )

				// TODO: pickup the duck
				.setTangent( Math.toRadians( 100 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( new Pose2d( -38, 66, Math.toRadians( 270 ) ), Math.toRadians( 90 ) )

				// drop duck in top
				.setTangent( Math.toRadians( -40 ) ) // direction to start next movement (line/spline)
				.splineToLinearHeading( MeepMeepPath.getHubPosition( 0, 270, 1+4/*robot.shippingHubDistance( RRHexBot.ShippingHubHeight.HIGH )*/, true ), Math.toRadians( 270 ) )
				.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1000 );
				} )
				.addTemporalMarker( ( ) -> {
//					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
//					robot.odometryLift.raise( );
				} )
				.waitSeconds( 1.2 )

				// TODO: move to barrier to park
				.setTangent( Math.toRadians( 90 ) )
				.splineToLinearHeading( new Pose2d( 11.5, 44, 0 ), Math.toRadians( -45 ) )
				.setVelConstraint( new MecanumVelocityConstraint( 50, 11.5 ) )
				.lineToLinearHeading( new Pose2d( 55, 44, 0 ) )

				.waitSeconds( 6 )


				.build( );
	}

}