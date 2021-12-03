package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.security.SecureClassLoader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

public class CubeWeightDetector extends OpenCvPipeline {

	Telemetry telemetry;
	Mat mat = new Mat( );

	public enum FreightType {
		WEIGHTED_CUBE,
		UNWEIGHTED_CUBE,
		NOT_CUBE
	}

	private FreightType freightType;


	static double FREIGHT_THRESHOLD = 0.3;
	static double WEIGHT_THRESHOLD = 0.1;

	public CubeWeightDetector( Telemetry t ) {
		telemetry = t;
	}

	public Mat processFrame( Mat input, String type ) {

		return processHSVColorFrame(input );
		//return processEdgeFrame( input );
	}

	public Mat processEdgeFrame( Mat src ) {
		Mat colorMask = new Mat(src.rows(), src.cols(), src.type());
		Mat gray = new Mat(src.rows(), src.cols(), src.type());
		Mat edges = new Mat(src.rows(), src.cols(), src.type());
		Mat dst = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));

//		Imgproc.cvtColor( src, colorMask, Imgproc.COLOR_RGB2HSV ); //f
//		Scalar lowHSV = new Scalar( 17, 0, 0 ); //f 17
//		Scalar highHSV = new Scalar( 55, 255, 255 ); //f 23
//		Core.inRange( colorMask, lowHSV, highHSV, colorMask);

		//Gray image
		Imgproc.cvtColor( src, gray, Imgproc.COLOR_RGB2GRAY);
		//Blur image from 1,1px to 5,5px
		Imgproc.blur(gray, edges, new Size(5,5));
		//Detect edge
		Imgproc.Canny(edges, edges, 25, 100);
		//copy detected edges to destination matrix
		src.copyTo(dst,edges);
		/*Mat srcCon = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchey = new Mat();
		Imgproc.findContours(srcCon, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		Iterator<MatOfPoint> it = contours.iterator();
		while(it.hasNext()) {
			telemetry.addLine(""+it.next());
		}*/
		return edges;
	}

	public Mat processHSVColorFrame( Mat input ) {
		Imgproc.cvtColor( input, mat, Imgproc.COLOR_RGB2HSV );//f
		//Imgproc.blur(mat, mat, new Size(3,3));
		Mat matWeight = mat.clone();

		Scalar freightLowHSV = new Scalar( 4, 0, 0 ); //f 17
		Scalar freightHighHSV = new Scalar( 29, 255, 255 ); //f 23
		Scalar weightLowHSV = new Scalar( 30, 10, 70 ); //w
		Scalar weightHighHSV = new Scalar( 100, 30, 255 ); //w

		SetupMatrix( mat, freightLowHSV, freightHighHSV );
		Rect BUCKET = new Rect ( 54, 0, 212, 240 );
		for( int i = 0; i < 10; i++ ) {
			Imgproc.morphologyEx( mat, mat, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement( Imgproc.MORPH_RECT, new Size( 2, 2 ) ) );
		}
		Rect FREIGHT_ROI = ScanMinArea( mat, BUCKET); //f
		Mat freightArea = mat.submat( FREIGHT_ROI );//f


		SetupMatrix( matWeight, weightLowHSV, weightHighHSV );
		Imgproc.morphologyEx( matWeight, matWeight, Imgproc.MORPH_DILATE, Imgproc.getStructuringElement( Imgproc.MORPH_RECT, new Size(3,3)));
		Rect WEIGHT_ROI = ScanMinArea( matWeight,FREIGHT_ROI );
		Mat weightArea = matWeight.submat( WEIGHT_ROI );

		double freightValue = Core.sumElems( freightArea ).val[0] / FREIGHT_ROI.area( ) / 255; //f
		double weightValue = Core.sumElems( weightArea ).val[0] / WEIGHT_ROI.area( ) / 255; //f

		telemetry.addLine("FV" + freightValue);
		telemetry.addLine("WV" + weightValue);

		telemetry.addLine("FROI X" + FREIGHT_ROI.x);
		telemetry.addLine("FROI Y" + FREIGHT_ROI.y);
		telemetry.addLine("WROI X" + WEIGHT_ROI.x);
		telemetry.addLine("WROI Y" + WEIGHT_ROI.y);

		freightArea.release( ); //f
		weightArea.release();

		boolean freightBool = freightValue > FREIGHT_THRESHOLD; //f
		boolean weightBool = weightValue > WEIGHT_THRESHOLD;

		Scalar red = new Scalar( 255, 0, 0 ); //f
		Scalar green = new Scalar( 0, 255, 0 ); //f
		Scalar blue = new Scalar( 0,0, 255);
		Scalar yellow = new Scalar(255, 255, 0);
		Scalar orange = new Scalar(255, 165, 0);
		Scalar pink = new Scalar (255, 0, 255);
		Scalar black = new Scalar( 0,0,0 );
		Scalar id = blue;
		if( freightBool & weightBool ) { //f
			freightType = FreightType.WEIGHTED_CUBE;
			telemetry.addLine( "Weighted Cube" );
			id = yellow;
		} else if( freightBool ) { //f
			freightType = FreightType.UNWEIGHTED_CUBE;
			telemetry.addLine( "Unweighted Cube" );
			id = orange;
		} else {
			freightType = FreightType.NOT_CUBE;
			telemetry.addLine( " Not Cube" );
		}




		Mat current = input;

//		Imgproc.cvtColor( current, current, Imgproc.COLOR_GRAY2RGB ); //i

		Imgproc.rectangle( current, BUCKET, id);
		Imgproc.rectangle( current, FREIGHT_ROI, freightType == FreightType.UNWEIGHTED_CUBE ? yellow : pink); //f
		Imgproc.rectangle( current, WEIGHT_ROI, freightType == FreightType.WEIGHTED_CUBE ? green : red); //f
		Rect BUCKETEDGE = new Rect ( 165, 0, 20 , 240 );
		Imgproc.rectangle( current, BUCKETEDGE, orange);

		return current;
	}

	@Override
	public Mat processFrame( Mat input ) {

		Mat cubeImage = processFrame( input, "cube" );
		double cubeVal = Core.sumElems( cubeImage ).val[0] / (cubeImage.rows( ) * cubeImage.cols( )) / 255;
		telemetry.update( );
		return cubeImage;
	}

	public FreightType getFreightType( ) {
		return freightType;
	}

	public Rect ScanMatrix (Mat mat) {
		return ScanMatrix(mat, new Rect(0,0,320,240));
		/*int minRow = mat.rows(), maxRow = 0, minCol = mat.cols(), maxCol = 0;

		for(int row = 0; row < mat.rows(); row++){
			for(int col = 0; col < mat.cols(); col++){
				double point = mat.get(row,col)[0];
				if(point == 255 & row < minRow) {
					minRow = row;
				}
				if(point == 255 & row > maxRow) {
					maxRow = row;
				}
				if(point == 255 & col < minCol) {
					minCol = col;
				}
				if(point == 255 & col > maxCol) {
					maxCol = col;
				}

			}
		}

		Rect CUBE_ROI = new Rect(
				new Point( minCol, minRow ),
				new Point( maxCol, maxRow ) );
		return CUBE_ROI;*/
	}

	public Rect ScanMatrix (Mat mat, Rect scanArea) {
		int minRow = mat.rows(), maxRow = 0, minCol = mat.cols(), maxCol = 0;

		for(int row = scanArea.y; row < scanArea.y + scanArea.height; row++){
			for(int col = scanArea.x; col < scanArea.x + scanArea.width; col++){
				if (mat.get(row,col) != null) {
					double point = mat.get( row, col )[0];
					if( point == 255 & row < minRow ) {
						minRow = row;
					}
					if( point == 255 & row > maxRow ) {
						maxRow = row;
					}
					if( point == 255 & col < minCol ) {
						minCol = col;
					}
					if( point == 255 & col > maxCol ) {
						maxCol = col;
					}
				}
			}
		}

		Rect CUBE_ROI = new Rect(
				new Point( minCol, minRow ),
				new Point( maxCol, maxRow ) );
		return CUBE_ROI;
	}

	public Rect ScanMinArea (Mat mat, Rect scanArea) {
		int minRow = 0, maxRow = mat.rows(), minCol = 0, maxCol = mat.cols();
		int maxRowCt = 0, maxColCt = 0, curRowCt = 0, curColCt = 0;
		for(int row = scanArea.y; row < scanArea.y + scanArea.height; row++){
			for(int col = scanArea.x; col < scanArea.x + scanArea.width; col++){
				if (mat.get(row,col) != null) {
					double point = mat.get( row, col )[0];
					if( point == 255) {
						curColCt++;
						if( maxColCt < curColCt) {
							maxColCt = curColCt;
							maxCol = col;
						}
					}
					else {
						curColCt = 0;
					}
				}
			}
		}
		for(int col = scanArea.x; col < scanArea.x + scanArea.width; col++) {
			for(int row = scanArea.y; row < scanArea.y + scanArea.height; row++) {
				if (mat.get(row,col) != null) {
					double point = mat.get( row, col )[0];
					if( point == 255) {
						curRowCt++;
						if( maxRowCt < curRowCt ) {
							maxRowCt = curRowCt;
							maxRow = row;
						}
					}
					else {
						curRowCt = 0;
					}
				}
			}
		}
		minRow = maxRow - maxRowCt - 1;
		minCol = maxCol - maxColCt - 1;
//		telemetry.addLine("("+minRow+","+minCol+") ");
//		telemetry.addLine("("+maxRow+","+maxCol+") ");
		minRow = minRow < scanArea.y ? scanArea.y : minRow;
		maxRow = maxRow > mat.rows() ? mat.rows() : maxRow;
		minCol = minCol < scanArea.x ? scanArea.x : minCol;
		maxCol = maxCol > mat.cols() ? mat.cols() : maxCol;
		Rect CUBE_ROI = new Rect(
				new Point( minCol, minRow ),
				new Point( maxCol, maxRow ) );
		return CUBE_ROI;
	}

	public void SetupMatrix(Mat mat, Scalar lowHSV, Scalar highHSV) {
		Core.inRange( mat, lowHSV, highHSV, mat );
	}
}