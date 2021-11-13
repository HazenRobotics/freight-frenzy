package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class AveragedGyro {

	BNO055IMU imu1;
	BNO055IMU imu2;
	List<LynxModule> hubs;

	public AveragedGyro( HardwareMap hardwareMap, String imu1Name, String imu2Name, BNO055IMU.Parameters params ) {
		imu1 = hardwareMap.get( BNO055IMU.class, imu1Name );
		imu2 = hardwareMap.get( BNO055IMU.class, imu2Name );
		hubs = hardwareMap.getAll( LynxModule.class );
		imu1.initialize( params );
		imu2.initialize( params );
	}

	public double getAngularHeading( ) {
		if( hubs.get( 0 ).isNotResponding( ) || hubs.get( 1 ).isNotResponding( ) ) {
			return imu1.getAngularOrientation( ).firstAngle;
		}
		double angle1 = imu1.getAngularOrientation( ).firstAngle;
		double angle2 = imu2.getAngularOrientation( ).firstAngle;
		double average = (angle1 + angle2) / 2;
		if( Math.signum( angle1 ) != Math.signum( angle2 ) ) {
			double normalizedAngle1 = Math.PI - Math.abs( angle1 );
			double normalizedAngle2 = Math.PI - Math.abs( angle2 );
			average = (Math.abs( angle1 ) + Math.abs( angle2 )) / 2;
			return normalizedAngle1 > normalizedAngle2 ? Math.signum( angle1 ) * average : Math.signum( angle2 ) * average;
		} else
			return average;
	}

	public double getAngularVelocity( ) {
		if( hubs.get( 0 ).isNotResponding( ) || hubs.get( 1 ).isNotResponding( ) ) {
			return imu1.getAngularVelocity( ).zRotationRate;
		}
		return (imu1.getAngularVelocity( ).zRotationRate + imu2.getAngularVelocity( ).zRotationRate) / 2;
	}


}
