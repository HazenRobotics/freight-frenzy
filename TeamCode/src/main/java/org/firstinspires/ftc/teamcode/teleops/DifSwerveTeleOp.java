package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.DifSwerveDrive;

@TeleOp(name = "Swerve Tele Op", group = "TeleOp")
public class DifSwerveTeleOp extends OpMode {

	DifSwerveDrive drive;

	final int LARGE_GEAR_TEETH = 102; // 6.008 / 2
	final int WHEEL_GEAR_TEETH = 25; // 1.637 / 2
	final double BELT_LARGE_GEAR_RADIUS = 6.531 / 2; // inches
	final double BELT_SMALL_GEAR_RADIUS = 1.0 / 4; // inches, small gear on the pulley
	final int BEVEL_GEAR_RATIO = 32 / 16;

	@Override
	public void init( ) {

		double rotateGearRatio = BEVEL_GEAR_RATIO * BELT_LARGE_GEAR_RADIUS / BELT_SMALL_GEAR_RADIUS;
		double wheelGearRatio = rotateGearRatio * LARGE_GEAR_TEETH / WHEEL_GEAR_TEETH;

		drive = new DifSwerveDrive( hardwareMap );
		drive.setUpWheelRatios( 3.9752, wheelGearRatio, rotateGearRatio, BELT_LARGE_GEAR_RADIUS );
		drive.setMaxAngularVelocity( Math.PI / 2 );
		drive.setWheelBase( 14 );

		DcMotorEx motor = hardwareMap.get( DcMotorEx.class, "frontLeft" );
		motor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		motor.setVelocity( 5 * (2 * Math.PI), AngleUnit.RADIANS );
		double startTime = getRuntime( );
		while( startTime + 5 > getRuntime( ) ) {
			telemetry.addLine( "Velocity: " + motor.getVelocity( AngleUnit.RADIANS ) );
			telemetry.update( );
		}
		motor.setVelocity( 0 );
	}

	@Override
	public void loop( ) {
		drive.move( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );
	}


}
