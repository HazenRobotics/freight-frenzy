package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;

public class WoodBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public MecanumDrive mecanumDrive;
	public EncoderTracker encoderTracker;

	public WoodBot( OpMode op ) {

		super( op );

		Robot.writeToDefaultFile( "Creating " + getClass( ).getSimpleName( ), false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes

//		new SoundLibrary( hardwareMap );

		mecanumDrive = new MecanumDrive( hardwareMap );
		mecanumDrive.setMotorDirections( Direction.REVERSE, Direction.REVERSE, Direction.FORWARD, Direction.FORWARD );
		super.driveTrain = mecanumDrive;
		encoderTracker = new EncoderTracker( hardwareMap, "frontLeft", "frontRight" );
	}

	/**
	 * @param time wait time in seconds
	 */
	public void sleepRobot( double time ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + time > opMode.getRuntime( ) ) ;
	}

}
