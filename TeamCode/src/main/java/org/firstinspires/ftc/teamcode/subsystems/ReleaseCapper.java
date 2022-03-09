package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ReleaseCapper {


	public enum Position {
		RELEASE,
		PICKUP,
		STORE
	}

	private final double RELEASE_POSITION = degreeToPercent( 0 );
	private final double PICKUP_POSITION = degreeToPercent( 45 );
	private final double STORE_POSITION = degreeToPercent( 270 );

	private Servo servo;
	private Position position;

	public ReleaseCapper( String capperName, HardwareMap hardwareMap ) {
		servo = hardwareMap.servo.get( capperName );
	}

	public void setPosition(Position position) {
		switch( position ) {
			case RELEASE:
				servo.setPosition( RELEASE_POSITION );
				break;
			case PICKUP:
				servo.setPosition( PICKUP_POSITION );
				break;
			case STORE:
				servo.setPosition( STORE_POSITION );
				break;
			default:
				throw new UnsupportedOperationException("You really messed up this time!");
		}
		this.position = position;
	}

	public Position getPosition() {
		return position;
	}

	private double degreeToPercent(double degree) {
		return degree / 270;
	}
}
