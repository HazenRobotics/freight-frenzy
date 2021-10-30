package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SquareIntake {

	DcMotor intakeMotorLeft;
	DcMotor intakeMotorRight;

	public SquareIntake( HardwareMap hw ) {
		setup( hw );
	}

	public void setup( HardwareMap hw ) {

		intakeMotorLeft = hw.dcMotor.get( "intakeLeft" );
		intakeMotorRight = hw.dcMotor.get( "intakeRight" );

		intakeMotorRight.setDirection( DcMotorSimple.Direction.REVERSE );
	}

	public void setPower( double power ) {

		intakeMotorLeft.setPower( power );
		intakeMotorRight.setPower( power );
	}

	public void eject( ) {
		setPower( -1 );
	}

	public void stop( ) {
		setPower( 0 );
	}

}