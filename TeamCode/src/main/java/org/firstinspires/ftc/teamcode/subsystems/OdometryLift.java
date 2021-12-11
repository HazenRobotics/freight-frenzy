package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometryLift {

	public CRServo odometryServo;

	public OdometryLift( HardwareMap hardwareMap ) {
		setup( hardwareMap, "odometryLift" );
	}

	public OdometryLift ( HardwareMap hardwareMap, String odometryServoName ) {
		setup( hardwareMap, odometryServoName );
	}

	public void setup( HardwareMap hardwareMap, String odometryServoName ) {
		odometryServo = hardwareMap.crservo.get( odometryServoName );
		odometryServo.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	public void liftOdometry( ) {
		setPowerTime( 1, 3000 );
	}

	public void lowerOdometry( ) {
		setPowerTime( -1, 3000 );
	}

	/**
	 *
	 * @param power - power to set the servo to for an amount of time
	 * @param time time to set power in milliseconds
	 */
	public void setPowerTime( double power, int time ) {
		new Thread( () -> {
			odometryServo.setPower( power );
			long startTime = System.currentTimeMillis();
			while( System.currentTimeMillis() < startTime + time );
			stop( );
		} ).start();
	}

	public void stop() {
		setPower( 0 );
	}

	public void setPower(double power) {
		odometryServo.setPower( power );
	}

	public double getPower( ) {
		return odometryServo.getPower( );
	}

}
