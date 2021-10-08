package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ExamplePipeline extends OpenCvPipeline  {

	@Override
	public Mat processFrame( Mat input ) {
		Mat output = input;
//		Imgproc.blur( input, output, new Size( 4, 4 ) );
		return output;
	}
}
