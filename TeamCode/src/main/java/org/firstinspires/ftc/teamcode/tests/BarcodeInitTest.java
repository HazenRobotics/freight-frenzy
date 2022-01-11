package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

@Autonomous(name = "New Barcode Test", group = "Test")
@Disabled
public class BarcodeInitTest extends LinearOpMode {

	@Override
	public void runOpMode( ) throws InterruptedException {

		BarcodeUtil detector = new BarcodeUtil( hardwareMap, "webcam1", telemetry );
		detector.init();

		telemetry.addLine( "OpenCv init finished" );
		telemetry.update( );

		waitForStart( );

		telemetry.addLine( "Position: " + detector.getBarcodePosition( ) );

		while( !isStopRequested() );

	}
}
