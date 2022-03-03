package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestTeleOp", group = "TeleOp")
public class TestTeleOp extends OpMode {

	@Override
	public void init( ) {

	}

	@Override
	public void loop( ) {

	}
}


/*



//*****************************************************************************
public class SimpleMouseTests extends Activity {

	int gnAPILevel;
	InputManager inputManager;
	boolean gfPtrVisibility;

	//***** For reflection checking of NVIDIA mouse extensions
	private static boolean gf_NVMouseExtensions;

	static {
		try { Wrap_NVMouseExtensions.checkAvailable();
			gf_NVMouseExtensions = true;
		}
		catch (Throwable t) { gf_NVMouseExtensions = false; }
	}

	//**************************************************************************
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		gnAPILevel = Build.VERSION.SDK_INT;
		gfPtrVisibility = true;
		inputManager = null;

		if (gnAPILevel >= Build.VERSION_CODES.JELLY_BEAN)  {
			inputManager = (InputManager) getSystemService(Context.INPUT_SERVICE);
		}
	}

	//**************************************************************************
	@Override
	public void onResume() {
		setMouseVisibility(gfPtrVisibility);
		super.onResume();
	}

	//**************************************************************************
	public void setMouseVisibility(boolean fVisibility) {
		if (! gf_NVMouseExtensions) return;
		gfPtrVisibility = fVisibility;
		Wrap_NVMouseExtensions.setCursorVisibility(inputManager, gfPtrVisibility);
	}

	//**************************************************************************
	@Override
	public boolean dispatchGenericMotionEvent(MotionEvent event) {
		int nGetAction = event.getActionMasked();

		if ((nSourceBits & InputDevice.SOURCE_MOUSE) == 0) {
			return super.dispatchGenericMotionEvent(event);
		}

		if (gf_NVMouseExtensions) {
			float flRelativeX =
					event.getAxisValue(Wrap_NVMouseExtensions.getAxisRelativeX(), 0);
			float flRelativeY =
					event.getAxisValue(Wrap_NVMouseExtensions.getAxisRelativeY(), 0);
		}

		return true;
	}
}

 */