package org.firstinspires.ftc.teamcode.vision;

/**
 * Author: Sam DePoule
 * <p>
 * Made using keyboard shortcuts ONLY
 */

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.vision.unused.Vuforia;

import java.util.ArrayList;

public class TensorFlowUtil {

	private final String TENSOR_FLOW_MODEL_NAME = "FreightFrenzy.tflite";

	public static final String[] LABELS = new String[]{ "Duck 1", "Duck 2", "Duck 3", "Element 1", "Element 2", "Element 3", "None" };

	private final Vuforia vuforia = Vuforia.getInstance( );

	// class specific
	private BarcodePosition[] recognitions;
	private ArrayList<BarcodePosition> infiniteRecognitions;

	// the default loops and how many loops we've done
	private int defaultLoops = 20000, totalLoops = 0;

	// how many of each stack type there are
	private int left = 0, center = 0, right = 0;

	// the start time of the stack detection method, and the final time it takes to loop through them
	private double startTime = 0, loopRunTime = 0;

	// # of rings in starting stack
	BarcodePosition position;

	TensorFlow tensorFlow;
	OpMode opMode;
	HardwareMap hardwareMap;

	public enum BarcodePosition {
		LEFT,
		CENTER,
		RIGHT
	}

	public TensorFlowUtil( OpMode op ) {
		opMode = op;
		hardwareMap = opMode.hardwareMap;
	}

	public void initTensorFlow( ) {
//
//		if( !vuforia.isRunning( ) ) {
//			final String VUFORIA_KEY = hardwareMap.appContext.getResources( ).getString( R.string.vuforia_key );
//			vuforia.setParameters( VUFORIA_KEY, "webcam1", true, hardwareMap );
//			vuforia.start( );
//		}

		tensorFlow = new TensorFlow( hardwareMap, "webcam1", TENSOR_FLOW_MODEL_NAME, 0.8f, LABELS );
	}

	void startTF( ) {
		tensorFlow.activate( );
	}

	BarcodePosition identifyObjects( ) {

		Recognition recognition = tensorFlow.getMostConfRecognition( );

		Log.d( "TFOD_TEST", "got recognitions: " + recognition );
		if( recognition != null ) {
			switch( recognition.getLabel( ) ) {
				case "Duck 1":
				case "Element 1": // Left
					return BarcodePosition.LEFT;
				case "Duck 2":
				case "Element 2": // Center
					return BarcodePosition.CENTER;
				case "Duck 3":
				case "Element 3": // Right
					return BarcodePosition.RIGHT;
			}
			Log.d( "TFOD_TEST", "tfod recognition name invalid: " + recognition.getLabel( )  );
		}

		Log.d( "TFOD_TEST", "didn't find a tfo recognition "  );

		return null;
	}

	void determineObjectLoop( int loops ) {

		Robot.writeToMatchFile( "objectDeterminationLoop", true );

		recognitions = new BarcodePosition[loops];

		resetLoopsAndCounters( );

		for( int i = 0; i < loops; i++ ) {
			recognitions[i] = identifyObjects( );

			Log.d( "TFOD_TEST", " identified an object"  );

			if( recognitions[i] == BarcodePosition.LEFT )
				left++;
			else if( recognitions[i] == BarcodePosition.CENTER )
				center++;
			else if( recognitions[i] == BarcodePosition.RIGHT )
				right++;

			opMode.telemetry.addLine( "stackRecognition #" + (totalLoops = i) + " : " + recognitions[i] );
			opMode.telemetry.update( );

//			if( left + center + right >= 5 )
//				break;
		}

		determinePositionFromCounts( );

		loopRunTime = opMode.getRuntime( ) - startTime;

		logAndPrint( position + " stack found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
	}

	void determineObjectWhileNotStartedSpeed( ) {
//        Robot.writeToMatchFile("objectDeterminationWhileLoop", true);

		infiniteRecognitions = new ArrayList<BarcodePosition>( );

		resetLoopsAndCounters( );

		BarcodePosition isRight = BarcodePosition.RIGHT;

		boolean doRightBreak = false;

		while( !((LinearOpMode) opMode).isStarted( ) || totalLoops < defaultLoops ) {

			isRight = identifyObjects( );
			if( isRight == BarcodePosition.RIGHT && doRightBreak )
				break;
			infiniteRecognitions.add( isRight );
			adjustStackCounts( infiniteRecognitions.get( totalLoops ), 1 );
			if( ++totalLoops > defaultLoops ) {
				adjustStackCounts( infiniteRecognitions.remove( 0 ), -1 );
				totalLoops--;
			}
		}

		determinePositionFromCounts( );

		if( isRight == BarcodePosition.RIGHT && doRightBreak )
			position = BarcodePosition.RIGHT;

		loopRunTime = opMode.getRuntime( ) - startTime;

		logAndPrint( position + " position found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
	}

	void determineObjectWhileNotStarted( ) {

		Robot.writeToMatchFile( "objectDeterminationWhileLoop", true );

		infiniteRecognitions = new ArrayList<>( );

		resetLoopsAndCounters( );

		while( !((LinearOpMode) opMode).isStarted( ) || totalLoops < defaultLoops ) {

			infiniteRecognitions.add( identifyObjects( ) );
			if( ++totalLoops > defaultLoops ) {
				infiniteRecognitions.remove( 0 );
				totalLoops--;
			}

			opMode.telemetry.addLine( "Total Loops: " + totalLoops );
			opMode.telemetry.update( );
		}

		for( int i = 0; i < totalLoops; i++ ) {
			adjustStackCounts( infiniteRecognitions.get( i ), 1 );
			opMode.telemetry.addLine( "positionRecognition #" + i + " : " + infiniteRecognitions.get( i ) );
			opMode.telemetry.update( );
		}

		determinePositionFromCounts( );

		loopRunTime = opMode.getRuntime( ) - startTime;

		logAndPrint( position + " position found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
	}

	private void resetLoopsAndCounters( ) {

		totalLoops = 0;
		left = 0;
		center = 0;
		right = 0;

		startTime = opMode.getRuntime( );
	}

	private void adjustStackCounts( BarcodePosition curPos, int adjustment ) {
		switch( curPos ) {

			case LEFT:
				left += adjustment;
				break;
			case CENTER:
				center += adjustment;
				break;
			case RIGHT:
				right += adjustment;
				break;
		}
	}

	private void determinePositionFromCounts( ) {

//		setPosition( BarcodePosition.CENTER );

		if( left > center && left > right )
			setPosition( BarcodePosition.LEFT );
		else if( center > left && center > right )
			setPosition( BarcodePosition.CENTER );
		else if( right > left && right > center )
			setPosition( BarcodePosition.RIGHT );
	}

	void stopTF( ) {
		tensorFlow.shutdown( );
	}

	public void deactivateTensorFlow( ) {
		stopTF( );
	}

	public void setPosition( BarcodePosition newPosition ) {
		this.position = newPosition;
	}


	public BarcodePosition getBarcodePosition( ) {
		return this.position;
	}

	public void setDefaultLoops( int newLoop ) {
		defaultLoops = newLoop;
	}

	public int getDefaultLoops( ) {
		return defaultLoops;
	}

	public int getTotalLoops( ) {
		return totalLoops;
	}

	public void setZoom( double zoom, double aspectRatio ) {
		tensorFlow.setZoom( zoom, aspectRatio );
	}

	public void setZoom( double zoom ) {
		tensorFlow.setZoom( zoom, 16.0 / 9.0 );
	}

	public void runPositionDetection( ) {
		Robot.writeToMatchFile( "runPostionDetection()", true );

		this.runPositionDetection( defaultLoops );
	}

	public void runPositionDetection( int loops ) {

		Robot.writeToMatchFile( "runPositionDetection( " + loops + " )", true );

		startTF( );

		determineObjectLoop( loops );

		stopTF( );
	}

	public void runWhileNotStartedStackDetection( ) {
		Robot.writeToMatchFile( "runWhileNotStartedStackDetection()", true );

		startTF( );

		determineObjectWhileNotStarted( );

		stopTF( );
	}

	public void runWhileNotStartedStackDetectionSpeed( ) {
//        Robot.writeToMatchFile("runWhileNotStartedStackDetectionSpeed()", true);

		startTF( );

		determineObjectWhileNotStartedSpeed( );

		stopTF( );
	}

	public void logAndPrint( String text, boolean includeTimeStamp ) {

//        Robot.writeToMatchFile(text, includeTimeStamp);
		opMode.telemetry.addLine( text );
		opMode.telemetry.update( );
	}

}
