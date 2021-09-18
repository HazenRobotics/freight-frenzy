package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class sets up and holds methods for running a mecanum drive
 */
public class MecanumDrive extends FourWheelDrive {

	final double WHEEL_DIAMETER = 1.4960629921; // odometry wheel 38mm
	// mecanum wheel diameter = 3.9788735773; circumference = 12.5

	/**
	 * Creates a MechanumDrive with default names for the wheels
	 *
	 * @param hw robot's hardware map
	 */
	public MecanumDrive( HardwareMap hw ) {
		super( hw );
	}

	/**
	 * Creates a MechanumDrive with specified names for the wheels
	 *
	 * @param hw                  robot's hardware map
	 * @param frontRightMotorName name of front right motor in the hardware map
	 * @param frontLeftMotorName  name of front left motor in the hardware map
	 * @param backRightMotorName  name of back right motor in the hardware map
	 * @param backLeftMotorName   name of back left motor in the hardware map
	 */
	public MecanumDrive( HardwareMap hw, String frontLeftMotorName, String frontRightMotorName, String backLeftMotorName, String backRightMotorName ) {
		super( hw, frontRightMotorName, frontLeftMotorName, backRightMotorName, backLeftMotorName );
	}

	public int convertDistTicks( double distanceToTravel ) {
		return convertDistTicks( distanceToTravel, Math.PI * WHEEL_DIAMETER );
	}

	public int convertTicksDist( double ticksToTravel ) {
		return convertTicksDist( ticksToTravel, Math.PI * WHEEL_DIAMETER );
	}


	/**
	 * Sets specified power to the motors
	 *
	 * @param frontRightPower power at which to run the front right motor.
	 * @param frontLeftPower  power at which to run the front left motor.
	 * @param backRightPower  power at which to run the back right motor.
	 * @param backLeftPower   power at which to run the back left motor.
	 */
	public void setMotorPower( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {
		super.setMotorPower( frontLeftPower, frontRightPower, backLeftPower, backRightPower );
	}

	/**
	 * Makes the robot strafe to the left or the right
	 *
	 * @param power power at which to strafe (+ is to the right, - is to the left)
	 */
	public void strafe( double power ) {
		drive( 0, 0, power );
	}

	/**
	 * Sets power to the wheel motors
	 *
	 * @param drive  power for forward and back motion
	 * @param strafe power for left and right robot
	 * @param rotate power for rotating the robot
	 */
	public void drive( double drive, double strafe, double rotate ) {

		// You might have to play with the + or - depending on how your motors are installed
		double frontLeftPower = drive + strafe - rotate;
		double backLeftPower = drive - strafe - rotate;
		double frontRightPower = drive - strafe + rotate;
		double backRightPower = drive + strafe + rotate;

		setMotorPower( frontLeftPower, frontRightPower, backLeftPower, backRightPower );
	}

    /*
    public void setMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }*/

}
