package com.example.meepmeeppaths;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public interface MeepMeepPath {

	static double robotLength = 13.25;
	static double robotWidth = 13.5; // with belts

	static final double tileSize = 23;
	static final double tileConnector = 0.75;

	static final double hubRadius = 9;

	static final double cameraRightIndent = 1.25;

	TrajectorySequence getTrajectorySequence( DriveShim drive );

	/**
	 * @param angle       the number of degrees to turn to reach the side of the shipping hub
	 * @param angleOffset the starting angle of the robot
	 * @param indent      the distance away from the shipping hub base to be
	 * @param blueSide    whether or not the robot is on the blue side
	 * @return the position (Pose2D) of where to go
	 */
	public static Pose2d getHubPosition( double angle, double angleOffset, double indent, boolean blueSide ) {
		double negate = Math.toRadians( angle * (blueSide ? 1 : -1) );
		double x = tileConnector / 2 + tileSize / 2 + Math.sin( negate ) * (hubRadius + indent + robotLength / 2);
		double y = tileConnector + tileSize + Math.cos( negate ) * (hubRadius + indent + robotLength / 2);
		return new Pose2d( -x, y * (blueSide ? 1 : -1), Math.toRadians( angleOffset + angle ) );
		// new Pose2d( -23.631, 35.506, toRadians( 270 + 45 ) )
	}


}
