package ca.mcgill.ecse211.color;

import lejos.hardware.Sound;
import ca.mcgill.ecse211.ecse211_project.*;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class ColorData {
	private SampleProvider colorSensor;
	private float [] colorData;
	private float red,green,blue;
	private int colorNumber = 200;
	private boolean found;
	private static final int BLUE = 1;
	private static final int GREEN = 2;
	private static final int YELLOW = 3;
	private static final int RED = 4;
	private static final float[] RED_RANGE = {0.90123f, 0.24171f,0.20345f};
	private static final float[] GREEN_RANGE = {0.28713f,0.83753f,0.46340f};
	private static final float[] BLUE_RANGE = {0.21072f,0.42964f,0.87800f};
	private static final float[] YELLOW_RANGE = {0.83059f,0.43110f,0.15239f};
	/**
	 * Color sensor is passed through constructor. 
	 * @param colorSensor
	 */
	public ColorData(SampleProvider colorSensor) {
		this.colorSensor = colorSensor;
		colorData = new float [colorSensor.sampleSize()];
	}

	/**
	 * This method returns the raw value from color sensor.
	 * @return
	 */
	public float[] getRGBData() {
		colorSensor.fetchSample(colorData, 0);
		red = colorData[0];
		green = colorData[1];
		blue = colorData[2];	
		return colorData;
	}

	/**
	 * This method returns the normalized color sensor data. Data are processed using the formula provided in the lab instruction
	 *  (i.e. R^ = R/(R^2+G^2+B^2)^0.5). 
	 * @return
	 */
	public float[] getNormalizedColorData() {
		float colorRatio[] = new float [3];
		getRGBData();
		colorRatio[0] = (float) (red/Math.sqrt(red*red + green*green + blue*blue));
		colorRatio[1] = (float) (green/Math.sqrt(red*red + green*green + blue*blue));
		colorRatio[2] = (float) (blue/Math.sqrt(red*red + green*green + blue*blue));
		return colorRatio;	
	}
	/**
	 * This method takes normalized color data and returns the mean of R,G and B value.
	 * @param data
	 * @return
	 */
	public float [] processData(float [][] data) {
		float [] mean = new float [3];
		float [] sum = new float [3];

		for(int i =0; i<colorNumber;i++) {
			for(int j =0; j<3;j++) {
				sum[j] += data[i][j];
			}
		}
		for(int k =0; k< 3;k++) {
			mean[k] = sum[k]/colorNumber;
		}
		return mean;

	}

	/**
	 * This method takes the mearsured RGB value and the calibrated RGB mean value,
	 *  and returns the Euclidean distance between them. 
	 * @param mearsuredRGB
	 * @param mean
	 * @return
	 */
	public double EDistance(float [] mearsuredRGB, float [] range) {
		double distance =
				Math.sqrt(Math.pow(mearsuredRGB[0] - range[0], 2)
						+Math.pow(mearsuredRGB[1] - range[1], 2)
						+Math.pow(mearsuredRGB[2] - range[2], 2));
		return distance;
	}

	/**
	 * This method returns the mean RGB value of a large amount of data points taken as an array.
	 * @return
	 */
	public float [] calibrate() {
		float [][] data = new float[colorNumber][3];
		for(int i =0; i< colorNumber;i++) {
			data[i] = getNormalizedColorData();
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		return processData(data);
	}
	/**
	 * This method takes 4 Euclidean distances and compare them. Then it returns the color name as a string.
	 * @param a
	 * @param b
	 * @param c
	 * @param d
	 * @return
	 */
	public String smallest(double red, double green,double blue,double yellow) {
		double first,second;
		String name = "ERROR!";
		first = (red<green)?red:green;
		second = (blue<yellow)?blue:yellow;
		first = (first<second)?first:second;
		name = (first == red)? "RED":name;
		name = (first == green)? "GREEN":name;
		name = (first == blue)? "BLUE":name;
		name = (first == yellow)? "YELLOW":name;
		return name;
	}
	/**
	 * This method takes 4 Euclidean distances and compare them. Then it returns the color name as an integer.
	 * @param red
	 * @param green
	 * @param blue
	 * @param yellow
	 * @return
	 */
	public int smallestInInt(double red, double green,double blue,double yellow) {
		double first,second;
		int colorID = -1;
		first = (red<green)?red:green;
		second = (blue<yellow)?blue:yellow;
		first = (first<second)?first:second;
		colorID = (first == red)? RED:colorID;
		colorID = (first == green)? GREEN:colorID;
		colorID = (first == blue)? BLUE:colorID;
		colorID = (first == yellow)? YELLOW:colorID;
		return colorID;
	}

	/**
	 * This method identidies a can's color by scanning 180 degrees around it and taking a large amount of data points.
	 * Then it computes euclidean distanes and compares them, prints the color name on lcd screen.
	 */
	public void identify() {
		LCD.clear();
		double redDist,greenDist,blueDist,yellowDist;
		float [] RGBData;
		Project.upMotor.setSpeed(180);
		LCD.drawString("Object Detected", 0, 0);
		Project.upMotor.rotate(180,true);
		RGBData = calibrate();
		redDist = EDistance(RGBData,RED_RANGE);
		yellowDist = EDistance(RGBData,YELLOW_RANGE);
		greenDist = EDistance(RGBData,GREEN_RANGE);
		blueDist = EDistance(RGBData,BLUE_RANGE);
		LCD.drawString(smallest(redDist,greenDist,blueDist,yellowDist), 0, 1);
		Project.upMotor.rotate(-180,false);
	}
	/**
	 * This method takes in an integer as the specified color ID.
	 * Then it identidies a can's color by scanning 180 degrees around it and taking a large amount of data points.
	 * Then it computes euclidean distanes and compares them, prints the color name on lcd screen.
	 * It will beep once if specified can is not found, beep twice other wise.
	 */
	public void identify(int colorID) {
		LCD.clear();
		double redDist,greenDist,blueDist,yellowDist;
		float [] RGBData;
		Project.upMotor.setSpeed(180);
		LCD.drawString("Object Detected", 0, 0);
		Project.upMotor.rotate(180,true);
		RGBData = calibrate();
		redDist = EDistance(RGBData,RED_RANGE);
		yellowDist = EDistance(RGBData,YELLOW_RANGE);
		greenDist = EDistance(RGBData,GREEN_RANGE);
		blueDist = EDistance(RGBData,BLUE_RANGE);
		int result = smallestInInt(redDist,greenDist,blueDist,yellowDist);
		LCD.drawString(smallest(redDist,greenDist,blueDist,yellowDist), 0, 1);		
		Project.upMotor.rotate(-180,false);
		if(result == colorID) {
			Sound.beep();
		}else {
			Sound.twoBeeps();
		}
		
	}


}
