package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.EncoderTracker;

import java.text.DecimalFormat;

@TeleOp(name = "EncoderPositionTest", group = "test")
//@Disabled
public class EncoderPositionTest extends OpMode {

	DcMotor[] encoder;
	int[] initialPosition;
	String[] names;

	DecimalFormat format;

	@Override
	public void init( ) {
		names = new String[]{ "frontLeft", "backLeft", "frontRight", "backRight", "lift", "spinner", "intake", "encoder" };
		encoder = new DcMotor[names.length];
		initialPosition = new int[encoder.length];
		for( int i = 0; i < names.length; i++ ) {
			encoder[i] = hardwareMap.dcMotor.get( names[i] );
			if( encoder[i] != null ) {
				encoder[i].setMode( DcMotor.RunMode.RUN_USING_ENCODER );
				initialPosition[i] = encoder[i].getCurrentPosition( );
			}
		}

		format = new DecimalFormat( "#.####" );

		telemetry.addLine( "Init Finished." );
		telemetry.update( );
	}

	@Override
	public void loop( ) {

		for( int i = 0; i < names.length; i++ ) { // 0, 1, 2, 3 are motors, 4, is lift, 5, & 6 are dead wheels

			if( encoder[i] != null ) {
				int ticksPosition = encoder[i].getCurrentPosition( ) - initialPosition[i];

				double inchesPosition;
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
		}

		telemetry.update( );
	}

}
