package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.localization.Field;
import org.firstinspires.ftc.teamcode.utils.Logger;

/**
 * This class sets up and manages a robot
 */
public abstract class Robot {

	HardwareMap hardwareMap;
	OpMode opMode;
	Telemetry telemetry;

	// drive
	public Drive driveTrain;

	/**
	 * Creates a Robot
	 *
	 * @param op robot's opMode
	 */
	public Robot( OpMode op ) {
		opMode = op;
		hardwareMap = opMode.hardwareMap;
		telemetry = opMode.telemetry;

		//vuforiaKey = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);

		//Bulk Caching to decrease cycle times
		for( LynxModule module : hardwareMap.getAll( LynxModule.class ) )
			module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );
	}

	public static void createMatchLogFile( String className ) {
		Logger.createMatchLogFile( className );
	}

	public static void createDefaultMatchLogFile() {
		Logger.writeToDefaultFile( "reated Default Log Fil", false, true );
	}

	/**
	 * writes to the default match file
	 *
	 * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
	 * @param includeTimeStamp will include the timeStamp for when the method is called
	 */
	public static void writeToMatchFile( String writeText, boolean includeTimeStamp ) {
		Logger.writeToMatchFile( writeText, includeTimeStamp );
	}

	/**
	 * writes to the default file *robotLog.txt
	 *
	 * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
	 * @param isAppending      true: will append to the file if it exists, false: will create a new file
	 * @param includeTimeStamp will include the timeStamp for when the method is called
	 */
	public static void writeToDefaultFile( String writeText, boolean isAppending, boolean includeTimeStamp ) {
		Logger.writeToDefaultFile( writeText, isAppending, includeTimeStamp );
	}

	/**
	 * @param fileName         the name of the file to write to
	 * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
	 * @param isAppending      true: will append to the file if it exists, false: will create a new file
	 * @param includeTimeStamp will include the timeStamp for when the method is called
	 */
	public static void writeAFile( String fileName, String writeText, boolean isAppending, boolean includeTimeStamp ) {
		Logger.writeAFile( fileName, writeText, isAppending, includeTimeStamp );
	}

	/**
	 * @param millis the amount of milliseconds to wait in a while loop
	 */
	public void sleep( long millis ) {
		long startTime = System.currentTimeMillis( );
		while( System.currentTimeMillis( ) < startTime + millis && opModeIsActive( ) ) ;
	}

	/**
	 * opMode version of LinearOpMode's opModeIsActive
	 *
	 * @return weather the OpMode is active or running
	 */
	public boolean opModeIsActive( ) {
		try {
			return ((LinearOpMode) opMode).opModeIsActive( );
		} catch( ClassCastException e ) {
			return true;
		}
	}

	/**
	 * @param robotLength the length of the robot
	 * @param angle       the number of degrees to turn to reach the side of the shipping hub
	 * @param angleOffset the starting angle of the robot
	 * @param indent      the distance away from the shipping hub base to be
	 * @param blueSide    whether or not the robot is on the blue side
	 * @return the position (Pose2D) of where the robot should move to fit the provided parameters
	 */
	public static Pose2d getHubPosition( double robotLength, double angle, double angleOffset, double indent, boolean blueSide ) {
		double negate = Math.toRadians( angle * (blueSide ? 1 : -1) );
		double x = Field.TILE_CONNECTOR / 2 + Field.TILE_SIZE / 2 + Math.sin( negate ) * (Field.HUB_RADIUS + indent + robotLength / 2);
		double y = Field.TILE_CONNECTOR + Field.TILE_SIZE + Math.cos( negate ) * (Field.HUB_RADIUS + indent + robotLength / 2);
		return new Pose2d( -x, y * (blueSide ? 1 : -1), Math.toRadians( angleOffset + angle ) );
		// ( -23.631, 35.506, toRadians( 270 + 45 ) )
	}

}
