package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.TimeZone;

public class Logger {

	private static final String LOG_PATH = "/" + "FIRST" + "/" + "logs" + "/";

	private static final String DEFAULT_ROBOT_LOG = "*robotLog.txt";
	private static final String DEFAULT_MATCH_LOG = "defaultRobotLog.txt";
	private static String matchLogFileName = DEFAULT_MATCH_LOG;

	private static final boolean deleteOldLogs = true;

	public static void createMatchLogFile( String className ) {

		SimpleDateFormat matchFormatter = new SimpleDateFormat( "MM-dd_HH'h'mm'm'_" );
		//SimpleDateFormat matchFormatter = new SimpleDateFormat( "MM-dd_HH-mm_" );
		matchFormatter.setTimeZone( TimeZone.getDefault( ) );
		String time = matchFormatter.format( new Date( ) );

		// will look like: 04-05_15h11m_TeleOpTechnicolor.txt
		// could look like: 04-05_15-11_TeleOpTechnicolor.txt

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
				time = new SimpleDateFormat( "MM-dd HH:mm:ss" ).format( date ) + " :: ";

			//".../Internal Storage";
			String path = Environment.getExternalStorageDirectory( ).getPath( ) + LOG_PATH;

			try {
				FileWriter writer = new FileWriter( new File( path + fileName ), isAppending );
				writer.write( time + writeText + System.lineSeparator( ) );
				writer.close( );
			} catch( IOException e ) {
				e.printStackTrace( );
			}

			if( deleteOldLogs )
				deleteOldLogsDay( new SimpleDateFormat( "MM-dd" ).format( date ) );
		} ).start( );

	}

	private static void deleteLog( String oldFileName ) {

		//".../Internal Storage";
		String path = Environment.getExternalStorageDirectory( ).getPath( ) + LOG_PATH;

		new File( path, oldFileName ).delete( );
	}

	/**
	 * deletes all files in the /FIRST/LOG/ directory without the date provided
	 *
	 * @param date the date to not delete - formatted like this: "08-31"
	 */
	public static void deleteOldLogsDay( String date ) {

		for( File f : Environment.getExternalStorageDirectory( ).listFiles( ) )
			if( !f.getName( ).startsWith( date ) )
				deleteLog( f.getName( ) );
	}

	/**
	 * deletes all files in the /FIRST/LOG/ directory without the date(s) provided
	 *
	 * @param dates the date(s) to not delete - formatted like this: "08-31"
	 */
	public static void deleteOldLogsDay( String[] dates ) {

		for( File f : Environment.getExternalStorageDirectory( ).listFiles( ) ) {
			boolean startsWithDate = false;
			for( int i = 0; i < dates.length; i++ )
				if( f.getName( ).startsWith( dates[i] ) )
					startsWithDate = true;
			if( !startsWithDate )
				deleteLog( f.getName( ) );
		}
	}

	/**
	 * deletes all files in the /FIRST/LOG/ directory without the date(s) provided
	 *
	 * @param dates the date(s) to not delete - formatted like this: "08-31"
	 */
	public static void deleteOldLogsDay( ArrayList<String> dates ) {

		for( File f : Environment.getExternalStorageDirectory( ).listFiles( ) ) {
			boolean startsWithDate = false;
			for( int i = 0; i < dates.size( ); i++ )
				if( f.getName( ).startsWith( dates.get( i ) ) )
					startsWithDate = true;
			if( !startsWithDate )
				deleteLog( f.getName( ) );
		}
	}


}
