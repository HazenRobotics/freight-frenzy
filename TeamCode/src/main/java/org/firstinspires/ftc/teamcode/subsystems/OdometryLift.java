package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometryLift {

	public CRServo odometryServo;

	public OdometryLift( HardwareMap hardwareMap ) {
		setup( hardwareMap, "odometryServo" );
	}

	public void setup( HardwareMap hardwareMap, String odometryServoName ) {
		odometryServo = hardwareMap.crservo.get( odometryServoName );
	}

	public void liftOdometry( ) {
		odometryServo.setPower( 0.5 );

	}

	public void lowerOdometry( ) {
		odometryServo.setPower( -0.5 );

	}

}
