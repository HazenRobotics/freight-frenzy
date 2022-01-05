package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.vision.CameraTargetDistance.componentDistanceFromTarget;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.vision.CameraTargetDistance;
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

						double width = recognition.getWidth( ), height = recognition.getHeight( );
						double x = recognition.getTop( );
						telemetry.addLine( "Top: " + x );
//						telemetry.addLine( "Bottom: " + recognition.getBottom( ) );
//						telemetry.addLine( "Left: " + recognition.getLeft( ) );
//						telemetry.addLine( "Right: " + recognition.getRight( ) );
						telemetry.addLine( "Width: " + width );
						telemetry.addLine( "Height: " + height );
						telemetry.addLine( "Confidence: " + recognition.getConfidence( ) );

						CameraTargetDistance.ObjectType objectType = CameraTargetDistance.ObjectType.DUCK;
						double distance = CameraTargetDistance.straightDistanceFromTarget( width, height, objectType );

						double camWidth = 720, camFOV = 70;
						double error = CameraTargetDistance.getLateralError( x, camWidth, camFOV );

						double cameraAngle = 22.5; // normal max of 22.5
						Vector2d finalDistance = componentDistanceFromTarget( distance, error, cameraAngle );

						double dist = CameraTargetDistance.distanceFromTargetSize( (width + height) / 2 );


						telemetry.addLine( "Distance 2 (from target): " + dist );
						telemetry.addLine( "Distance (from target): " + distance );
						telemetry.addLine( "Lateral Error (from target): " + error );
						telemetry.addLine( "Final Distance (from target): " + finalDistance );


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
