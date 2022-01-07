package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.vision.CameraTargetDistance.componentDistanceFromTarget;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.CameraTargetDistance;
import org.firstinspires.ftc.teamcode.vision.TensorFlowUtilBack;

import java.text.DecimalFormat;
import java.util.List;

@TeleOp(name = "VuforiaModelTest", group = "Test")
//@Disabled
public class VuforiaModelTest extends LinearOpMode {

	TensorFlowUtilBack tensorFlowUtil;
	DecimalFormat fm = new DecimalFormat( "0.###" );

	@Override
	public void runOpMode( ) throws InterruptedException {

		tensorFlowUtil = new TensorFlowUtilBack( this );
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
						telemetry.addLine( "Label Name: " + labelName + " (" + fm( recognition.getConfidence( ) ) + ")" );
						telemetry.addLine( "---------" );

						// top, bottom, left, right, height, width, confidence

						double width = recognition.getWidth( ), height = recognition.getHeight( );

						double top = recognition.getTop( ), left = recognition.getLeft( ), bottom = recognition.getBottom( ), right = recognition.getRight( );
						telemetry.addLine( "Top: " + fm( top ) );
						telemetry.addLine( "Bottom: " + fm( bottom ) );
						telemetry.addLine( "Left: " + fm( left ) );
						telemetry.addLine( "Right: " + fm( right ) );
						telemetry.addLine( "Width: " + fm( width ) );
//						telemetry.addLine( "Height: " + height );

						double camWidth = 720, camFOV = 45;
						double error = CameraTargetDistance.getLateralError( left + (width / 2), camWidth, camFOV );

						double dist = CameraTargetDistance.parDistFromTargetSize( width );

						double cameraAngle = 90 - 22.5;
						Vector2d finalDistance = componentDistanceFromTarget( dist, error, cameraAngle );

						telemetry.addLine( "Distance (from target): " + fm( dist ) );
						telemetry.addLine( "Lateral Error (from target): " + fm( error ) );
						telemetry.addLine( "Final Distance (from target): " + finalDistance );

					} else {
						telemetry.addLine( "no recognitions: " + fm( getRuntime( ) ) );
					}

					telemetry.addLine( );
					telemetry.addLine( "=============================================" );

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

	public String fm( double input ) {
		return fm.format( input );
	}

}
