package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.EncoderTracker;

import java.text.DecimalFormat;

//import org.firstinspires.ftc.teamcode.utils.Tracking;

@TeleOp(name = "EncoderPositionTest", group = "test")
@Disabled
public class EncoderPositionTest extends OpMode {

	DcMotor[] encoder;
	int[] initialPosition;
	String[] names;

	DecimalFormat format;

	@Override
	public void init( ) {
		names = new String[]{ "frontLeft", "backLeft", "frontRight", "backRight", "lift", "intake", "perpendicular" };
		encoder = new DcMotor[names.length];
		initialPosition = new int[encoder.length];
		for( int i = 0; i < names.length; i++ ) {
			encoder[i] = hardwareMap.dcMotor.get( names[i] );
			encoder[i].setMode( DcMotor.RunMode.RUN_USING_ENCODER );
			initialPosition[i] = encoder[i].getCurrentPosition( );
		}

		format = new DecimalFormat( "#.####" );

		telemetry.addLine( "Init Finished." );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		for( int i = 0; i < names.length; i++ ) { // 0, 1, 2, 3 are motors, 4, is lift, 5, & 6 are dead wheels

			int ticksPosition = encoder[i].getCurrentPosition( ) - initialPosition[i];

			double inchesPosition = 0;
			if( i < 4 ) // 0, 1, 2, 3, are motors
				inchesPosition = EncoderTracker.convertTicksDist( ticksPosition, 2 * 50 / 25.4, 537.6, 1 );
			else if( i > 4 ) // 5 & 6 are dead wheels
				inchesPosition = EncoderTracker.convertTicksDist( ticksPosition, 2 * 50 / 25.4, 480, 30 / 48.0 ) / 4.0;
			else // 4 is the lift
				inchesPosition = EncoderTracker.convertTicksDist( ticksPosition, 32 / 25.4, 537.6, 1 );
			String extraSpace = inchesPosition < 0 ? "" : " ";

			telemetry.addLine( names[i] + " current position:" );
			telemetry.addLine( "  " + ticksPosition + " ticks" );
			telemetry.addLine( "  " + extraSpace + format.format( inchesPosition ) + " inches" );
		}

		telemetry.update( );
	}

	public String roundDouble( double input, int decimalPlaces ) {
 		// -50.98765, 3
		int negate = input < 0 ? -1 : 1; // negate = -1
		input *= negate; // input = 50.98765
		int inputShort = (int) input; // inputShort = 50
		input -= inputShort; // input = 0.98765
		String output = ("" + input); // output = "0.98765"
		// 50 + "."
		return (negate * inputShort) + output.substring( 1, output.length( ) - 2 >= decimalPlaces ? decimalPlaces : output.length( ) );
	}

}
