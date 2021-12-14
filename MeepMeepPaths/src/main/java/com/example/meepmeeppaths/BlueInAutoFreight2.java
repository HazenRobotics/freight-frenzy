package com.example.meepmeeppaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class BlueInAutoFreight2 implements MeepMeepPath{

	double wallPos = 64.624;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 17, 64.25, Math.toRadians( 270 ) ) )
				.setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint( 90 ), new MecanumVelocityConstraint( 60, 17 ) ) ) )

				// move to dump initial block in designated layer
				.addTemporalMarker( ( ) -> {
//					robot.liftToShippingHubHeight( height );
				} )
				.setTangent( Math.toRadians( 180 ) )
				.splineToLinearHeading( MeepMeepPath.getHubPosition( -22.5, 270, 1.5, true ), Math.toRadians( 270 - 22.5 ) )
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )

				.waitSeconds( 0.8 )

				// move to grab block 1
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 48, wallPos ) ) // 48
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
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 2
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18/*49*/, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 50, wallPos ) ) // 53
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
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 0.8 )

				// move to grab block 3
				.setTangent( Math.toRadians( 90 ) )
				.splineToSplineHeading( new Pose2d( 18, wallPos, Math.toRadians( 180 ) ), Math.toRadians( 0 ) )
				.addTemporalMarker( ( ) -> {
//					robot.intake.setPower( 0.6 );
				} )
				.lineToConstantHeading( new Vector2d( 52, wallPos ) ) // 50
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
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.addTemporalMarker( ( ) -> {
//					robot.drive.setDeadwheelsDisabledCheck( ( ) -> true );
//					robot.odometryLift.liftOdometry( );
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
