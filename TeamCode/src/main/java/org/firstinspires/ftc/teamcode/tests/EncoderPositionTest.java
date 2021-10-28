package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.EncoderTracker;

import java.text.DecimalFormat;

//import org.firstinspires.ftc.teamcode.utils.Tracking;

@TeleOp(name = "EncoderPositionTest", group = "test")
//@Disabled
public class EncoderPositionTest extends OpMode {

	DcMotor[] encoder;
	int[] initialPosition;
	String[] names;

	DecimalFormat format;

	@Override
	public void init( ) {
		names = new String[]{ "frontLeft", "backLeft", "frontRight", "backRight", "lift", "intake", "perpendicular" };
		encoder = new DcMotor[names.length];
//		encoder[0] = hardwareMap.dcMotor.get( "frontLeft" );
//		encoder[1] = hardwareMap.dcMotor.get( "backLeft" );
//		encoder[2] = hardwareMap.dcMotor.get( "frontRight" );
//		encoder[3] = hardwareMap.dcMotor.get( "backRight" );
//		encoder[4] = hardwareMap.dcMotor.get( "lift" );
//		encoder[5] = hardwareMap.dcMotor.get( "intake" );
//		encoder[6] = hardwareMap.dcMotor.get( "perpendicular" );
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

		for( int i = 0; i < names.length; i++ ) {

			int ticksPosition = encoder[i].getCurrentPosition( ) - initialPosition[i];
			// get rid of this when Andrew fixes next encoder
//			ticksPosition /= 8;
			double inchesPosition = EncoderTracker.convertTicksDist( ticksPosition, 50 / 25.4, 480, 30 / 48.0 ) / 4.0;
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
