package org.firstinspires.ftc.teamcode.teleops;

import android.hardware.input.InputManager;
import android.view.MotionEvent;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

//*****************************************************************************
public class Wrap_NVMouseExtensions {

	private static Method mInputManager_setCursorVisibility;
	private static int nMotionEvent_AXIS_RELATIVE_X = 0;
	private static int nMotionEvent_AXIS_RELATIVE_Y = 0;


	//**************************************************************************
	static {
		try {
			mInputManager_setCursorVisibility = InputManager.class.getMethod( "setCursorVisibility", boolean.class );
		} catch( NoSuchMethodException ex ) {
			throw null/* some exception */;
		}

		try {
			Field fieldMotionEvent_AXIS_RELATIVE_X =
					MotionEvent.class.getField( "AXIS_RELATIVE_X" );
			try {
				nMotionEvent_AXIS_RELATIVE_X = (Integer) fieldMotionEvent_AXIS_RELATIVE_X.get( null );
			} catch( IllegalAccessException ex ) {
				throw null/* some exception */;
			}
		} catch( NoSuchFieldException ex ) {
			throw null/* some exception */;
		}


		/* DO THE SAME FOR RELATIVEY */
	}

	//**************************************************************************
	public static void checkAvailable( ) { /* force initialization above */ }

	//**************************************************************************
	public static boolean setCursorVisibility( InputManager im, boolean fVisibility ) {
		try {
			mInputManager_setCursorVisibility.invoke( im, fVisibility );
		} catch( InvocationTargetException ite ) {
			return false;
		} catch( IllegalAccessException iae ) {
			return false;
		}

		return true;
	}

	//**************************************************************************
	public static int getAxisRelativeX( ) {
		return nMotionEvent_AXIS_RELATIVE_X;
	}

	public static int getAxisRelativeY( ) {
		return nMotionEvent_AXIS_RELATIVE_Y;
	}

}