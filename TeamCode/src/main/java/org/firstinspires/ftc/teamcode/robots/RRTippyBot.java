package org.firstinspires.ftc.teamcode.robots;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocket;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.RRMecanumDriveTippy42;
import org.firstinspires.ftc.teamcode.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.Capper;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSpinnerMotor;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OdometryLift;
import org.firstinspires.ftc.teamcode.utils.EncoderTracker;
import org.firstinspires.ftc.teamcode.utils.TargetPositionCalculator;
import org.firstinspires.ftc.teamcode.vision.BarcodePositionDetector;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;
import org.firstinspires.ftc.teamcode.vision.TensorFlowUtilBack;

public class RRTippyBot extends Robot {

	public OpMode opMode;
	public HardwareMap hardwareMap;

	public BarcodeUtil barcodeUtil;

	public RRMecanumDriveTippy42 drive;
	public MecanumDrive mecanumDrive;
	public EncoderTracker encoderTracker;

	public CarouselSpinnerMotor spinner;

	public Lift lift;
	public Bucket bucket;
	public Capper capper;
	public Grabber grabber;

	public OdometryLift odometryLift;

	public Intake intake;

	TensorFlowUtilBack duckTensorFlow;
	TargetPositionCalculator calculator;
	private Vector2d lastIdentified = null;
	private boolean searchForDuck = true;

	public static final double LIFT_ANGLE = 50;

	public static final double BUCKET_ANGLE_RANGE = 167.5; // 90 - -77.5 = 167.5
	// lift default is 35, max of 165, range = 200

	public static final double BUCKET_ANGLE_INTAKE = 90; // theoretically should be exactly 90 but 0.0 - 0.4 doesn't set position correctly on the servo
	public static final double BUCKET_ANGLE_MOVING = LIFT_ANGLE;
	public static final double BUCKET_ANGLE_DUMP = -25; // should be around -45 but the servo is weird

	public static final double CAPPER_PICKUP = 1.0;
	public static final double CAPPER_HOLD = 0.8;

	public static double ROBOT_LENGTH = 13.5;
	public static double ROBOT_WIDTH = 12.5;

	public RRTippyBot( OpMode op, boolean auto ) {

		super( op );

		Robot.writeToDefaultFile( "Creating " + getClass( ).getSimpleName( ), false, false );

		opMode = op;
		hardwareMap = op.hardwareMap;

		// initialize util objects/classes

//		new SoundLibrary( hardwareMap );

		if( auto )
			drive = new RRMecanumDriveTippy42( hardwareMap );

		spinner = new CarouselSpinnerMotor( hardwareMap );

		lift = new Lift( hardwareMap, "lift", false, 2.375, (38.2 / 25.4) / 2, LIFT_ANGLE, AngleUnit.DEGREES );
		// LIFT_ANGLE - 90 :: because the servo's one position is below and perpendicular to the lift
		bucket = new Bucket( hardwareMap, "bucket", BUCKET_ANGLE_INTAKE - BUCKET_ANGLE_RANGE, BUCKET_ANGLE_RANGE );
		capper = new Capper( hardwareMap, "capper" );
		grabber = new Grabber( hardwareMap, "grabber" );

		mecanumDrive = new MecanumDrive( hardwareMap );
		// bevel gear madness
		mecanumDrive.setMotorDirections( Direction.REVERSE, Direction.FORWARD, Direction.REVERSE, Direction.REVERSE );
		super.driveTrain = mecanumDrive;
		encoderTracker = new EncoderTracker( hardwareMap, "frontLeft", "frontRight", 38 / 25.4, 537.7, 1 );

		odometryLift = new OdometryLift( hardwareMap, "odometryLift", true );

		intake = new Intake( hardwareMap );

		if( auto ) {

			barcodeUtil = new BarcodeUtil( hardwareMap, "webcam1", telemetry );

			duckTensorFlow = new TensorFlowUtilBack( opMode );
			calculator = new TargetPositionCalculator( new Vector2d( -13 - 9, 8 - 3 ) );
		}

		capper.setPosition( 0 );

	}

	public void initTF( ) {
		duckTensorFlow.initTensorFlow( );
		duckTensorFlow.startTF( );
	}

	public void stopTF() {
		duckTensorFlow.stopTF();
	}

	public void waitForDuck( ) {

		double timeStarted = opMode.getRuntime( );

		telemetry.addLine( "About to wait" );
		telemetry.update( );

		while( opModeIsActive( ) && searchForDuck && !duckTensorFlow.isActive( ) && timeStarted < opMode.getRuntime( ) + 5 ) {


			telemetry.addLine( "Waiting for tensorflow to activate or 5 seconds" );
			telemetry.update( );

			try {
				Thread.sleep( 100 );
			} catch( InterruptedException ignored ) {
			}
		}
		telemetry.update( );
	}

	/**
	 * @param interval how often to recognize objects (milliseconds)
	 */
	public void startDuckScanning( int interval ) {
		new Thread( ( ) -> {

			while( opModeIsActive( ) && searchForDuck && !duckTensorFlow.isActive( ) ) {
				telemetry.addLine( "Scanning will start once tensorflow is activated" );
				telemetry.update( );
			}

			telemetry.addLine( "Started Duck Scanning." );
			telemetry.update( );

			while( opModeIsActive( ) && searchForDuck ) {

				Recognition recognition = duckTensorFlow.identifyObject( );

				if( recognition != null ) {
					Vector2d position = calculator.getTargetPosition( recognition, drive.getExternalHeading( ) );

					if( position != null )
						lastIdentified = position;
				}

				try {
					Thread.sleep( interval );
				} catch( InterruptedException ignored ) {
				}
			}
		} ).start( );
	}

	public void stopDuckScanning( ) {
		searchForDuck = false;
	}

	/**
	 * @return the field position of the last identified object (will only be null if it hasn't found any recognitions yet)
	 */
	public Vector2d getDuckPosition( ) {

		if( lastIdentified == null ) {
			Pose2d pos = drive.getPoseEstimate( );
			return new Vector2d( pos.getX( ) + 0.1, pos.getY( ) + 0.1);
		}

		return new Vector2d( lastIdentified.getX( ) + drive.getPoseEstimate().getX(), lastIdentified.getY( ) + drive.getPoseEstimate().getY() );
	}

	/**
	 * @param angle the pose heading angle to return
	 * @return the field position of the last identified object (if null returns current position)
	 */
	public Pose2d getDuckPosition( double angle ) {

		if( lastIdentified == null ) {
			Pose2d pos = drive.getPoseEstimate( );
			return new Pose2d( pos.getX( ) + 0.1, pos.getY( ) + 0.1, pos.getHeading( ) );
		}

		return new Pose2d( lastIdentified.getX( ) + drive.getPoseEstimate().getX(), lastIdentified.getY( ) + drive.getPoseEstimate().getY(), angle );
	}

	/**
	 * @param robotAngle angle of the robot (radians)
	 * @param minLoops   number of required (non-null) loops to get recognitions
	 * @return the field position of the identified object (returns null if: it doesn't find a target in @minLoops or in under 1 second)
	 */
	public Vector2d getDuckPosition( double robotAngle, int minLoops, double intakePower ) {

		Recognition delayed = null;

		double waitedTime = opMode.getRuntime( );

		for( int i = 0; i < minLoops; ) {
			delayed = duckTensorFlow.identifyObject( ); // (most confident recognition)
			if( delayed != null )
				i++;
			else if( waitedTime > 1 )
				break;
		}

		if( delayed == null )
			return null;

		intake.setPower( intakePower );

		return calculator.getTargetPosition( delayed, robotAngle );
	}

	/**
	 * @param time wait time in seconds
	 */
	public void sleepRobot( double time ) {
		double startTime = opMode.getRuntime( );
		while( opModeIsActive( ) && startTime + time > opMode.getRuntime( ) ) {
			try {
				Thread.sleep( 50 );
			} catch( InterruptedException ignored ) {
			}
		}
	}

	public double getLongitudinalPosition( ) {
		return encoderTracker.convertTicksDist( encoderTracker.getLongitudinalPosition( ) );
	}

	public double getLateralPosition( ) {
		return encoderTracker.convertTicksDist( encoderTracker.getLateralPosition( ) );
	}

	public double shippingHubHeightToInches( RRHexBot.ShippingHubHeight height ) {

		telemetry.addLine( "shippingHubHeightToInches: " + height );
		telemetry.update( );
		switch( height ) {
			default: // (LOW)
				return 8.5;
			case MIDDLE:
				return 13;
			case HIGH:
				return 20;
		}
	}

	public void liftToShippingHubHeight( RRHexBot.ShippingHubHeight height ) {
		bucket.setAngle( RRTippyBot.BUCKET_ANGLE_MOVING );
		lift.setHeightVelocity( 1200, shippingHubHeightToInches( height ) );
	}

	public RRHexBot.ShippingHubHeight barcodePosToShippingHubHeight( BarcodePositionDetector.BarcodePosition position ) {
		switch( position ) {
			default: // (LEFT)
				return RRHexBot.ShippingHubHeight.LOW;
			case MIDDLE:
				return RRHexBot.ShippingHubHeight.MIDDLE;
			case RIGHT:
				return RRHexBot.ShippingHubHeight.HIGH;
		}
	}

	public double shippingHubDistance( RRHexBot.ShippingHubHeight height ) {
		switch( height ) {
			default: // (LOW)
				return 2; // 1.5
			case MIDDLE:
				return 3; // 5
			case HIGH:
				return 7;
		}
	}

	public void dumpBucket( ) {
		bucket.setAngle( BUCKET_ANGLE_DUMP );
		sleepRobot( 0.65 );
		bucket.setAngle( BUCKET_ANGLE_MOVING );
//		bucket.setAngle( BUCKET_ANGLE_INTAKE );
	}

	/*public double distanceFromShippingHub( RRHexBot.ShippingHubHeight height) {
		switch( height ) {
			case LOW:
				return 10.5 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
			case MIDDLE:
				return 7.5 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
			case HIGH:
				return 6 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
			default:
				return 11 + lift.calcBucketDistanceFromHeight( shippingHubHeightToInches( height ));
		}
	}*/

	/**
	 * @param angle       the number of degrees to turn to reach the side of the shipping hub
	 * @param angleOffset the starting angle of the robot
	 * @param indent      the distance away from the shipping hub base to be
	 * @param blueSide    whether or not the robot is on the blue side
	 * @return the position (Pose2D) of where the robot should move to fit the provided parameters
	 */
	public static Pose2d getHubPosition( double angle, double angleOffset, double indent, boolean blueSide ) {
		return Robot.getHubPosition( ROBOT_LENGTH, angle, angleOffset, indent, blueSide );
	}

}
