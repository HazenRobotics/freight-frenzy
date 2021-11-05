package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drives.Drive;
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

}
