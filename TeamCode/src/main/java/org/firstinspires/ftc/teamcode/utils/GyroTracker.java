package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class GyroTracker {

	BNO055IMU gyro1;
	BNO055IMU gyro2;
	HardwareMap hardwareMap;
	Orientation angles;
	Acceleration gravity;

	public GyroTracker( HardwareMap hardwareMap ) {
		this( hardwareMap, false );
	}

	public GyroTracker( HardwareMap hardwareMap, boolean twoGyros ) {

		initGyro( hardwareMap, twoGyros );
	}

	public void initGyro( HardwareMap hardwareMap, boolean twoGyros ) {

		this.hardwareMap = hardwareMap;

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters( );
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator( );

		gyro1 = hardwareMap.get( BNO055IMU.class, "imu" );
		gyro1.initialize( parameters );

		if( twoGyros ) {
			gyro2 = hardwareMap.get( BNO055IMU.class, "imu2" );
			gyro2.initialize( parameters );
		}

		angles = gyro1.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
	}

	/**
	 * convert a distance (in inches) to ticks
	 *
	 * @param distance            the distance you want to convert to ticks
	 * @param wheelDiameter       the diameter of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the number of ticks in that distance
	 */
	public static int convertDistTicks( double distance, double wheelDiameter, double pulsesPerRevolution, double gearRatio ) {
		double revolutions = distance / Math.PI * wheelDiameter;
		int totalTicks = (int) Math.round( (revolutions * pulsesPerRevolution) / gearRatio );

		return totalTicks;
	}

	/**
	 * convert a number of ticks to distance (in inches)
	 *
	 * @param ticks               the ticks you want to convert to distance
	 * @param wheelDiameter       the diameter of the encoder wheel
	 * @param pulsesPerRevolution the encoder's pulses per revolution
	 * @param gearRatio           the ratio of the gears
	 * @return the distance (in inches) in that number of ticks
	 */
	public static double convertTicksDist( double ticks, double wheelDiameter, double pulsesPerRevolution, double gearRatio ) {
		double circumference = Math.PI * wheelDiameter;
		double totalDistance = (ticks * circumference * gearRatio) / pulsesPerRevolution;

		return totalDistance;
	}

	// *************** all gyro methods ***************

	public double getGyroXVelocity( ) {
		return gyro1.getVelocity( ).xVeloc;
	}

	public double getGyroYVelocity( ) {
		return gyro1.getVelocity( ).yVeloc;
	}

	public double getGyroZVelocity( ) {
		return gyro1.getVelocity( ).zVeloc;
	}

	public float getGyroHeading( ) {
		angles = gyro1.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
		return angles.firstAngle;
		//return formatAngle(angles.angleUnit, angles.firstAngle);
	}

	public float getGyroRoll( ) {
		angles = gyro1.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
		return angles.secondAngle;
		//return formatAngle(angles.angleUnit, angles.secondAngle);
	}

	public float getGyroPitch( ) {
		angles = gyro1.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
		return angles.thirdAngle;
	}

	public float get360GyroHeading( ) {
		return (-getGyroHeading( ) + 360) % 360;
	}

	/**
	 * will sleep the robot for [millis] milliseconds
	 *
	 * @param millis
	 */
	public void sleep( long millis ) {
		long startTime = System.currentTimeMillis( );
		while( System.currentTimeMillis( ) < startTime + millis ) ;
	}


}
