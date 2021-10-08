package org.firstinspires.ftc.teamcode.utils;

/**
 * Author: Sam DePoule
 * <p>
 * Made using keyboard shortcuts ONLY
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.Robot;

import java.util.ArrayList;

public class TensorFlowUtil {

	private final String TENSOR_FLOW_MODEL_NAME = "UltimateGoal.tflite";

	private static final String LABEL_FIRST_ELEMENT = "Quad";
	private static final String LABEL_SECOND_ELEMENT = "Single";

	private final Vuforia vuforia = Vuforia.getInstance( );

	// class specific
	private Stack[] stackRecognitions;
	private ArrayList<Stack> infiniteStackRecognitions;

	// the default loops and how many loops we've done
	private int defaultLoops = 20000, totalLoops = 0;

	// how many of each stack type there are
	private int singles = 0, quads = 0;

	// the start time of the stack detection method, and the final time it takes to loop through them
	private double startTime = 0, loopRunTime = 0;

	// # of rings in starting stack
	Stack stack;

	TensorFlow tensorFlow;
	OpMode opMode;
	HardwareMap hardwareMap;

	public enum Stack {
		NONE,
		SINGLE,
		QUAD
	}

	public TensorFlowUtil( OpMode op ) {
		opMode = op;
		hardwareMap = opMode.hardwareMap;
	}

	public void initTensorFlow( ) {

		if( !vuforia.isRunning( ) ) {
			final String VUFORIA_KEY = hardwareMap.appContext.getResources( ).getString( R.string.vuforia_key );
			vuforia.setParameters( VUFORIA_KEY, "webcam", true, hardwareMap );
			vuforia.start( );
		}

		tensorFlow = new TensorFlow( TENSOR_FLOW_MODEL_NAME, 0.8f, true, hardwareMap, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT );
	}

	void startTF( ) {
		tensorFlow.activate( );
	}

	Stack identifyObjects( ) {
		Recognition recognition = tensorFlow.getRecognition( );
		if( recognition != null ) {
			switch( recognition.getLabel( ) ) {
				case LABEL_SECOND_ELEMENT: // "Single"
					return Stack.SINGLE;
				case LABEL_FIRST_ELEMENT: // "Quad"
					return Stack.QUAD;
			}
		}
		return Stack.NONE;
	}

	void determineObjectLoop( int loops ) {
		Robot.writeToMatchFile( "objectDeterminationLoop", true );

		stackRecognitions = new Stack[loops];

		resetLoopsAndCounters( );

		for( int i = 0; i < loops; i++ ) {
			stackRecognitions[i] = identifyObjects( );
			if( stackRecognitions[i] == Stack.SINGLE )
				singles++;
			else if( stackRecognitions[i] == Stack.QUAD )
				quads++;

			opMode.telemetry.addLine( "stackRecognition #" + (totalLoops = i) + " : " + stackRecognitions[i] );
			opMode.telemetry.update( );

			if( singles + quads >= 5 )
				break;
		}

		determineStackFromCounts( );

		loopRunTime = opMode.getRuntime( ) - startTime;

		logAndPrint( stack + " stack found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
	}

	void determineObjectWhileNotStartedSpeed( ) {
//        Robot.writeToMatchFile("objectDeterminationWhileLoop", true);

		infiniteStackRecognitions = new ArrayList<Stack>( );

		resetLoopsAndCounters( );

		Stack isQuad = Stack.NONE;

		boolean doQuadBreak = false;

		while( !((LinearOpMode) opMode).isStarted( ) || totalLoops < defaultLoops ) {

			isQuad = identifyObjects( );
			if( isQuad == Stack.QUAD && doQuadBreak )
				break;
			infiniteStackRecognitions.add( isQuad );
			adjustStackCounts( infiniteStackRecognitions.get( totalLoops ), 1 );
			if( ++totalLoops > defaultLoops ) {
				adjustStackCounts( infiniteStackRecognitions.remove( 0 ), -1 );
				totalLoops--;
			}
		}

		determineStackFromCounts( );

		if( isQuad == Stack.QUAD && doQuadBreak )
			stack = Stack.QUAD;

		loopRunTime = opMode.getRuntime( ) - startTime;

		logAndPrint( stack + " stack found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
	}

	void determineObjectWhileNotStarted( ) {
		Robot.writeToMatchFile( "objectDeterminationWhileLoop", true );

		infiniteStackRecognitions = new ArrayList<Stack>( );

		resetLoopsAndCounters( );

		while( !((LinearOpMode) opMode).isStarted( ) || totalLoops < defaultLoops ) {

			infiniteStackRecognitions.add( identifyObjects( ) );
			if( ++totalLoops > defaultLoops ) {
				infiniteStackRecognitions.remove( 0 );
				totalLoops--;
			}

			opMode.telemetry.addLine( "Total Loops: " + totalLoops );
			opMode.telemetry.update( );
		}

		for( int i = 0; i < totalLoops; i++ ) {
			adjustStackCounts( infiniteStackRecognitions.get( i ), 1 );
			opMode.telemetry.addLine( "stackRecognition #" + i + " : " + infiniteStackRecognitions.get( i ) );
			opMode.telemetry.update( );
		}

		determineStackFromCounts( );

		loopRunTime = opMode.getRuntime( ) - startTime;

		logAndPrint( stack + " stack found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
	}

	private void resetLoopsAndCounters( ) {

		totalLoops = 0;
		singles = 0;
		quads = 0;

		startTime = opMode.getRuntime( );
	}

	private void adjustStackCounts( Stack curStack, int adjustment ) {
		switch( curStack ) {
			case SINGLE:
				singles += adjustment;
				break;
			case QUAD:
				quads += adjustment;
				break;
		}
	}

	private void determineStackFromCounts( ) {
		setStack( Stack.NONE );
		if( singles > quads )
			setStack( Stack.SINGLE );
		else if( quads > singles )
			setStack( Stack.QUAD );
	}

	void stopTF( ) {
		tensorFlow.shutdown( );
	}

	public void deactivateTensorFlow( ) {
		stopTF( );
	}

	public void setStack( Stack newStack ) {
		this.stack = newStack;
	}

	public Stack getStack( ) {
		return this.stack;
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

	public void runStackDetection( ) {
		Robot.writeToMatchFile( "runStackDetection()", true );

		runStackDetection( defaultLoops );
	}

	public void runStackDetection( int loops ) {
		Robot.writeToMatchFile( "runStackDetection( " + loops + " )", true );

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
