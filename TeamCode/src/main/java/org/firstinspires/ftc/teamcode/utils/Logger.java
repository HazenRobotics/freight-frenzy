package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;
import android.util.Log;

import com.sun.tools.doclint.Env;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;
import java.util.Objects;
import java.util.TimeZone;

public class Logger {

	private static final String LOG_PATH = "/" + "FIRST" + "/" + "logs" + "/";

	private static final String DEFAULT_ROBOT_LOG = "*robotLog.txt";
	private static final String DEFAULT_MATCH_LOG = "defaultRobotLog.txt";
	private static String matchLogFileName = DEFAULT_MATCH_LOG;

	private static final boolean deleteOldLogs = true;

	public static void createMatchLogFile( String className ) {

		SimpleDateFormat matchFormatter = new SimpleDateFormat( "MM-dd_HH-mm_", Locale.getDefault( ) );
		matchFormatter.setTimeZone( TimeZone.getDefault( ) );
		String time = matchFormatter.format( new Date( ) );

		// ("MM-dd_HH'h'mm'm'_") looks like: 04-05_15h11m_TeleOpTechnicolor.txt
		// ("MM-dd_HH-mm_") looks like: 04-05_15-11_TeleOpTechnicolor.txt

		matchLogFileName = time + className + ".txt";
		writeAFile( matchLogFileName, matchLogFileName + ": created", false, true );
	}

	/**
	 * writes to the default match file
	 *
	 * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
	 * @param includeTimeStamp will include the timeStamp for when the method is called
	 */
	public static void writeToMatchFile( String writeText, boolean includeTimeStamp ) {
		Log.v( "MatchFile", writeText );
		writeAFile( matchLogFileName, writeText, true, includeTimeStamp );
	}

	/**
	 * writes to the default file *robotLog.txt
	 *
	 * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
	 * @param isAppending      true: will append to the file if it exists, false: will create a new file
	 * @param includeTimeStamp will include the timeStamp for when the method is called
	 */
	public static void writeToDefaultFile( String writeText, boolean isAppending, boolean includeTimeStamp ) {
		Log.v( "DefaultFile", writeText );
		writeAFile( DEFAULT_ROBOT_LOG, writeText, isAppending, includeTimeStamp );
	}

	/**
	 * @param fileName         the name of the file to write to
	 * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
	 * @param isAppending      true: will append to the file if it exists, false: will create a new file
	 * @param includeTimeStamp will include the timeStamp for when the method is called
	 */
	public static void writeAFile( String fileName, String writeText, boolean isAppending, boolean includeTimeStamp ) {

		// "\n" = System.lineSeparator()

		new Thread( ( ) -> {

			String time = "";
			Date date;
			if( includeTimeStamp || deleteOldLogs )
				date = new Date( );
			if( includeTimeStamp )
				time = new SimpleDateFormat( "MM-dd HH:mm:ss", Locale.getDefault( ) ).format( date ) + " :: ";

			//".../Internal Storage";
			String path = Environment.getExternalStorageDirectory( ).getPath( ) + LOG_PATH;

			try {
				FileWriter writer = new FileWriter( path + fileName, isAppending );
				writer.write( time + writeText + System.lineSeparator( ) );
				writer.close( );
			} catch( IOException e ) {
				e.printStackTrace( );
			}

			if( deleteOldLogs )
				deleteOldLogsDay( new SimpleDateFormat( "MM-dd", Locale.getDefault( ) ).format( date ) );
		} ).start( );

	}

	private static boolean deleteLog( String oldFileName ) {

		//".../Internal Storage";
		String path = Environment.getExternalStorageDirectory( ).getPath( ) + LOG_PATH;

		return new File( path, oldFileName ).delete( );
	}

	/**
	 * deletes all files in the /FIRST/LOG/ directory without the date provided
	 *
	 * @param date the date to not delete - formatted like this: "08-31"
	 */
	public static void deleteOldLogsDay( String date ) {


//		File dir = Environment.getExternalStorageDirectory( );
		File dir = Environment.getRootDirectory( );
//		File dir = Environment.getDataDirectory( );

		for( File file : Objects.requireNonNull( dir.listFiles( ) ) )
//			if( !file.getName( ).startsWith( date ) )
				Log.e( "LOG", "" + file.getName( ) + System.lineSeparator( ) + " :: " + file.getPath( ) + System.lineSeparator( ) + " :: " + file.getAbsolutePath( ) );
//				new File( dir.getPath( ) + LOG_PATH, file.getName( ) );
//				deleteLog( f.getName( ) );
	}

	/**
	 * deletes all files in the /FIRST/LOG/ directory without the date(s) provided
	 *
	 * @param dates the date(s) to not delete - formatted like this: "08-31"
	 * @return an array with which files were deleted (true = success)
	 */
	public static boolean[] deleteOldLogsDay( String[] dates ) {

		return deleteOldLogsDay( (ArrayList<String>) Arrays.asList( dates ) );

//		File dir = Environment.getExternalStorageDirectory( );
//
//		boolean[] successfulOperations = new boolean[dates.length];
//		int index = 0;
//
//		for( File file : Objects.requireNonNull( dir.listFiles( ) ) ) {
//			boolean startsWithDate = false;
//			for( String date : dates )
//				if( file.getName( ).startsWith( date ) )
//					startsWithDate = true;
//			if( !startsWithDate )
//				successfulOperations[index++] = new File( dir.getPath( ) + LOG_PATH, file.getName( ) ).delete( );
//		}
//
//		return successfulOperations;
	}

	/**
	 * deletes all files in the /FIRST/LOG/ directory without the date(s) provided
	 *
	 * @param dates the date(s) to not delete - formatted like this: "08-31"
	 * @return an array with which files were deleted (true = success)
	 */
	public static boolean[] deleteOldLogsDay( ArrayList<String> dates ) {

		File dir = Environment.getExternalStorageDirectory( );

		boolean[] successfulOperations = new boolean[dates.size( )];
		int index = 0;

		for( File file : Objects.requireNonNull( dir.listFiles( ) ) ) {
			boolean startsWithDate = false;
			for( int i = 0; i < dates.size( ); i++ )
				if( file.getName( ).startsWith( dates.get( i ) ) )
					startsWithDate = true;
			if( !startsWithDate )
				successfulOperations[index++] = new File( dir.getPath( ) + LOG_PATH, file.getName( ) ).delete( );
		}

		return successfulOperations;
	}


}
