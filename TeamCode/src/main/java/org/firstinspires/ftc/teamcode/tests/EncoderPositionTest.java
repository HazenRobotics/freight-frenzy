package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.Tracking;

@TeleOp(name = "EncoderPositionTest", group = "test")
//@Disabled
public class EncoderPositionTest extends OpMode {

	DcMotor encoder;
	int initialPosition;

	@Override
	public void init( ) {

		encoder = hardwareMap.dcMotor.get( "frontLeft" );
		initialPosition = encoder.getCurrentPosition( );

		telemetry.addLine( "Init Finished." );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		int ticksPosition = encoder.getCurrentPosition( ) - initialPosition;
		// get rid of this when Andrew fixes next encoder
		ticksPosition /= 8;
		double inchesPosition = Tracking.convertTicksDist( ticksPosition, 1.4961, 250, 1 );

		telemetry.addLine( "Current Position:" );
		telemetry.addLine( "  " + ticksPosition + " ticks" );
		telemetry.addLine( "  " + inchesPosition + " inches" );
		telemetry.update( );
	}

}
