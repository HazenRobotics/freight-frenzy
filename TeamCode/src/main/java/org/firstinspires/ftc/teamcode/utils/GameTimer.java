package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.util.Timing.Timer;

import java.util.concurrent.TimeUnit;

public class GameTimer {

	private static long offset = 0;

	private static Timer timer = new Timer(158, TimeUnit.SECONDS);

	public static long remainingTime() {
		return timer.remainingTime() - offset;
	}

	public static long elapsedTime() {
		return timer.elapsedTime() + offset;
	}

	public static void start() {
		timer.start();
	}

	public static void startFromTeleop() {
		offset = 38;
		start();
	}

	public static void pause() {
		timer.pause();
	}

	public static void resume() {
		timer.resume();
	}

	public static boolean done() {
		return timer.done();
	}

	public static boolean isTimerOn() {
		return timer.isTimerOn();
	}

	public static long remainingTimeAutonomous() {
		return 30 - elapsedTime();
	}

	public static long remainingTimeTeleop() {
		return 128 - elapsedTime();
	}

	public static long remainingTimeEndgame() {
		return 158 - elapsedTime();
	}

	public static boolean inEndgame() {
		return elapsedTime() > 128;
	}

	public static boolean inTeleop() {
		return elapsedTime() > 38;
	}

}
