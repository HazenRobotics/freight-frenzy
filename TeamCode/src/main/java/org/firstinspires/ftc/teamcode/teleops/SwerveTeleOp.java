package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drives.SwerveDrive;

@TeleOp(name = "Swerve Tele Op", group = "TeleOp")
public class SwerveTeleOp extends OpMode {

	SwerveDrive drive;

	DcMotorEx leftTopMotor;
	DcMotorEx leftBottomMotor;
	DcMotorEx rightTopMotor;
	DcMotorEx rightBottomMotor;

	final int LARGE_GEAR_TEETH = 102; // 6.008 / 2
	final int WHEEL_GEAR_TEETH = 25; // 1.637 / 2
	final double BELT_LARGE_GEAR_RADIUS = 6.531 / 2;
	final double BELT_SMALL_GEAR_RADIUS = 1.0 / 4; // small gear on the pulley
	final int BEVEL_GEAR_RATIO = 32 / 16;

	@Override
	public void init( ) {

		double wheelGearRatio = BEVEL_GEAR_RATIO * (BELT_LARGE_GEAR_RADIUS / BELT_SMALL_GEAR_RADIUS) * ((double) LARGE_GEAR_TEETH / WHEEL_GEAR_TEETH);
		double rotateGearRatio = BEVEL_GEAR_RATIO * (BELT_LARGE_GEAR_RADIUS / BELT_SMALL_GEAR_RADIUS);

		drive = new SwerveDrive( hardwareMap, wheelGearRatio, rotateGearRatio, BELT_LARGE_GEAR_RADIUS );

		leftTopMotor = hardwareMap.get( DcMotorEx.class, "leftTop" );
		leftBottomMotor = hardwareMap.get( DcMotorEx.class, "leftBottom" );
		rightTopMotor = hardwareMap.get( DcMotorEx.class, "rightTop" );
		rightBottomMotor = hardwareMap.get( DcMotorEx.class, "rightBottom" );

	}

	@Override
	public void loop( ) {
		robotOrientedDrive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
		robotOrientedDrive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
	}


	public void robotOrientedDrive( double drive, double strafe, double rotate ) {

		// set robot
		double stickDeadZone = 0.05;
		if( Math.abs( drive + strafe ) < stickDeadZone ) {
			// do nothing
			this.drive.stop( );
		} else {
			// drive
		}


	}

	public void fieldOrientedDrive( double drive, double strafe, double rotate ) {

	}

	public int convertDistTicks( double distanceToTravel, double circumference ) {
		return drive.convertDistTicks( distanceToTravel, circumference, drive.wheelGearRatio );
	}

	public int convertTicksDist( double ticksToTravel, double circumference ) {
		return drive.convertTicksDist( ticksToTravel, circumference, drive.wheelGearRatio );
	}

	public int convertRotTicks( double radiansToTravel, double circumference ) {
		return drive.convertDistTicks( radiansToTravel, circumference, drive.rotateGearRatio );
	}

	public int convertTicksRot( double ticksToTravel, double circumference ) {
		return drive.convertTicksDist( ticksToTravel, circumference, drive.rotateGearRatio );
	}

	public void rotateWheelPod( double power, DcMotorEx topMotor, DcMotorEx bottomMotor ) {
		// topMotor +1 pow rotates top clockwise
		// bottomMotor +1 pow rotates bottom counterclockwise
		topMotor.setPower( power );
		bottomMotor.setPower( -power );
		// 500 500 - moved 500 ticks
		// 500 -500 - rotated 500 ticks

		// -250 -250 - moved -250 ticks
		// 500 -500 - rotated 500 ticks
		// 250 -750
	}

	public void rotateWheelPods( double power ) {
		rotateWheelPod( power, leftBottomMotor, leftTopMotor );
		rotateWheelPod( power, rightBottomMotor, rightTopMotor );
	}

	public void moveWheel( double power, DcMotorEx topMotor, DcMotorEx bottomMotor ) {
		// topMotor +1 pow rotates top clockwise
		// bottomMotor +1 pow rotates bottom counterclockwise
		topMotor.setPower( power );
		bottomMotor.setPower( power );
	}
}
