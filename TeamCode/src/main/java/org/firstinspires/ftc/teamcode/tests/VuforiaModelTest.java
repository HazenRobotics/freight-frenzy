package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.TensorFlowUtil2;

import java.util.List;

@TeleOp(name = "VuforiaModelTest", group = "Test")
//@Disabled
public class VuforiaModelTest extends LinearOpMode {

	TensorFlowUtil2 tensorFlowUtil;

	@Override
	public void runOpMode( ) throws InterruptedException {


		tensorFlowUtil = new TensorFlowUtil2( this );
		tensorFlowUtil.initTensorFlow( );
		tensorFlowUtil.startTF( );

		telemetry.addLine( "Init finished!" );
		telemetry.update( );

		while( !isStopRequested( ) || !isStarted( ) ) {

			List<Recognition> recognitions = tensorFlowUtil.identifyObjects( );

			if( recognitions != null ) {

				for( int i = 0; i < recognitions.size( ); i++ ) {

					Recognition recognition = recognitions.get( i );
					if( recognition != null ) {
						String labelName = recognition.getLabel( );
						telemetry.addLine( "Label Name: " + labelName );
						telemetry.addLine( );

						// top, bottom, left, right, height, width, confidence

						telemetry.addLine( "Top: " + recognition.getTop( ) );
						telemetry.addLine( "Bottom: " + recognition.getBottom( ) );
						telemetry.addLine( "Left: " + recognition.getLeft( ) );
						telemetry.addLine( "Right: " + recognition.getRight( ) );
						telemetry.addLine( "Height: " + recognition.getHeight( ) );
						telemetry.addLine( "Width: " + recognition.getWidth( ) );
						telemetry.addLine( "Confidence: " + recognition.getConfidence( ) );
					} else {
						telemetry.addLine( "no recognitions: " + getRuntime( ) );
					}

					telemetry.addLine( "-------------------------" );

				}

				telemetry.update( );
			}
		}

		waitForStart( );


		telemetry.addLine( "Closing TensorFlow..." );
		telemetry.update( );

		tensorFlowUtil.stopTF( );

		telemetry.addLine( "TensorFlow Closed." );
		telemetry.update( );

	}

}
