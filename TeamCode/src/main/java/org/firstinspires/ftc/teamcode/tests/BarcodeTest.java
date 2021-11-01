package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.unused.TensorFlowUtil;

@Autonomous(name = "Barcode Test", group = "Test")
public class BarcodeTest extends LinearOpMode {

	TensorFlowUtil tensorFlow;


	@Override
	public void runOpMode( ) throws InterruptedException {

		Log.e( "TFOD_TEST", "run opmode: " );

		tensorFlow = new TensorFlowUtil( this );
		tensorFlow.initTensorFlow( );

		telemetry.addLine( "TensorFlow init finished" );
		telemetry.update( );

		waitForStart( );

		Log.e( "TFOD_TEST", "started opmode: " );

		telemetry.addLine( "Running position detection..." );
		telemetry.update( );

		tensorFlow.runPositionDetection( );

		telemetry.addData( "Detected barcode position", tensorFlow.getBarcodePosition( ) );
		telemetry.update( );

		while( !isStopRequested( ) ) ;

	}
}
