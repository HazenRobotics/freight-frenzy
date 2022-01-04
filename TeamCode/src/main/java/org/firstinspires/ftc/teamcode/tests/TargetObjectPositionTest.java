package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

@Autonomous()
public class TargetObjectPositionTest extends LinearOpMode {


	// Adjust these numbers to suit your robot.
	final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
	//  The GAIN constants set the relationship between the measured position error,
	//  and how much power is applied to the drive motors.  Drive = Error * Gain
	//  Make these values smaller for smoother control.
	final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
	final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

	final double MM_PER_INCH = 25.40;   //  Metric conversion

	VuforiaLocalizer vuforia = null;
	OpenGLMatrix targetPose = null;
	String targetName = "";

//	private DcMotor leftDrive = null;
//	private DcMotor rightDrive = null;

	MecanumDrive mecanumDrive;

	@Override
	public void runOpMode( ) throws InterruptedException {

		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 * To get an on-phone camera preview, use the code below.
		 * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
		 */
		int cameraMonitorViewId = hardwareMap.appContext.getResources( ).getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName( ) );
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters( cameraMonitorViewId );
		// VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = hardwareMap.appContext.getResources( ).getString( R.string.vuforia_key );

		// Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
		parameters.useExtendedTracking = false;

		// Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
		parameters.cameraName = hardwareMap.get( WebcamName.class, "webcam" );
		this.vuforia = ClassFactory.getInstance( ).createVuforia( parameters );

		// Load the trackable objects from the Assets file, and give them meaningful names
		VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset( "full_model_with_metadata" );
		targetsFreightFrenzy.get( 0 ).setName( "b" );
		targetsFreightFrenzy.get( 1 ).setName( "d" );
		targetsFreightFrenzy.get( 2 ).setName( "e" );
		targetsFreightFrenzy.get( 3 ).setName( "h" );
		targetsFreightFrenzy.get( 4 ).setName( "l" );
		targetsFreightFrenzy.get( 5 ).setName( "m" );

		// Start tracking targets in the background
		targetsFreightFrenzy.activate( );

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
//		leftDrive = hardwareMap.get( DcMotor.class, "left_drive" );
//		rightDrive = hardwareMap.get( DcMotor.class, "right_drive" );
		mecanumDrive = new MecanumDrive( hardwareMap );

		// To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
		// Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//		leftDrive.setDirection( DcMotor.Direction.FORWARD );
//		rightDrive.setDirection( DcMotor.Direction.REVERSE );

		telemetry.addData( ">", "Press Play to start" );
		telemetry.update( );

		waitForStart( );

		boolean targetFound = false;    // Set to true when a target is detected by Vuforia
		double targetRange = 0;        // Distance from camera to target in Inches
		double targetBearing = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
		double drive = 0;        // Desired forward power (-1 to +1)
		double turn = 0;        // Desired turning power (-1 to +1)

		while( opModeIsActive( ) ) {
			// Look for first visible target, and save its pose.
			targetFound = false;
			for( VuforiaTrackable trackable : targetsFreightFrenzy ) {
				if( ((VuforiaTrackableDefaultListener) trackable.getListener( )).isVisible( ) ) {
					targetPose = ((VuforiaTrackableDefaultListener) trackable.getListener( )).getVuforiaCameraFromTarget( );

					// if we have a target, process the "pose" to determine the position of the target relative to the robot.
					if( targetPose != null ) {
						targetFound = true;
						targetName = trackable.getName( );
						VectorF trans = targetPose.getTranslation( );

						// Extract the X & Y components of the offset of the target relative to the robot
						double targetX = trans.get( 0 ) / MM_PER_INCH; // Image X axis
						double targetY = trans.get( 2 ) / MM_PER_INCH; // Image Z axis

						// target range is based on distance from robot position to origin (right triangle).
						targetRange = Math.hypot( targetX, targetY );

						// target bearing is based on angle formed between the X axis and the target range line
						targetBearing = Math.toDegrees( Math.asin( targetX / targetRange ) );

						break;  // jump out of target tracking loop if we find a target.
					}
				}
			}

			// Tell the driver what we see, and what to do.
			if( targetFound ) {
				telemetry.addData( ">", "HOLD Left-Bumper to Drive to Target\n" );
				telemetry.addData( "Target", " %s", targetName );
				telemetry.addData( "Range", "%5.1f inches", targetRange );
				telemetry.addData( "Bearing", "%3.0f degrees", targetBearing );
			} else {
				telemetry.addData( ">", "Drive using joystick to find target\n" );
			}

			// Drive to target Automatically if Left Bumper is being pressed, AND we have found a target.
			if( gamepad1.left_bumper && targetFound ) {

				// Determine heading and range error so we can use them to control the robot automatically.
				double rangeError = (targetRange - DESIRED_DISTANCE);
				double headingError = targetBearing;

				// Use the speed and turn "gains" to calculate how we want the robot to move.
				drive = rangeError * SPEED_GAIN;
				turn = headingError * TURN_GAIN;

				telemetry.addData( "Auto", "Drive %5.2f, Turn %5.2f", drive, turn );
			} else {

				// drive using manual POV Joystick mode.
				drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
				turn = gamepad1.right_stick_x / 4.0;  // Reduce turn rate to 25%.
				telemetry.addData( "Manual", "Drive %5.2f, Turn %5.2f", drive, turn );
			}
			telemetry.update( );

			// Calculate left and right wheel powers and send to them to the motors.
//			double leftPower = Range.clip( drive + turn, -1.0, 1.0 );
//			double rightPower = Range.clip( drive - turn, -1.0, 1.0 );
//			leftDrive.setPower( leftPower );
//			rightDrive.setPower( rightPower );
			mecanumDrive.drive( drive, 0, turn );

			sleep( 10 );
		}
	}
}
