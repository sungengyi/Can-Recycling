package ca.mcgill.ecse211.odometer;

/*
 * OdometryCorrection.java
 */


import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.BaseSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_SIZE = 30.48;
	private static final double WHEEL_RAD = 2.1;
	private static final double C_WHEEL = 2 * WHEEL_RAD * Math.PI;

	


	private Odometer odometer;
	private SampleProvider lightSensor;
	private float [] data;





	/**
	 * This is the default class constructor. An existing instance of the odometer is used. This is to
	 * ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}

	public OdometryCorrection(Odometer odometer, SampleProvider lightsensor, float [] data) {
		this.odometer = odometer;
		this.lightSensor=lightsensor;
		this.data = data;


	}




	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		
		double intensity;
		
		odometer.setXYT(0, 0, 0);
		int changeY1 = -1,changeX1 = -1;
//		int changeY2 = 2, changeX2 = 2;
		
		while (true) {
			correctionStart = System.currentTimeMillis();

			lightSensor.fetchSample(data, 0);
			intensity = data[0];

//			if(intensity < 0.30 ) {
//				switch(odometer.getCounter()) {
//				case 0:
//					 changeY1++;
//					 odometer.setY((changeY1) * TILE_SIZE);
//					
//					 odometer.setTheta(0);
//					 Sound.beep();
//					 break;
//				case 1:
//					 changeX1++;
//					 odometer.setX((changeX1 ) * TILE_SIZE);	
//					
//					 odometer.setTheta(90);
//					 Sound.beep();
//					 break;
//				
//				case 2:	
//					
//					odometer.setY((changeY1) * TILE_SIZE);
//					changeY1--;
//					odometer.setTheta(180);
//					Sound.beep();
//					break;
//				case 3:
//					
//					 odometer.setX((changeX1) * TILE_SIZE);
//					 changeX1--;
//					 odometer.setTheta(270);
//					 Sound.beep();
//					 break;
//				default:
//					//do nothing
//				}
//			}else {
//				
//			}
//			 if(theta > 0 && theta <= 95) {
//				 
//			 }
//			 else if (theta > 95 && theta <= 185) {
//				 
//			 }
//			 else if(theta > 185 && theta <= 275) {
//				changeY--;
//				odometer.setY((changeY - 1) * TILE_SIZE);
//				odometer.setTheta(270);
//				Sound.beep();
//			
//			 }
//			 else if(theta > 275 && theta <= 360) {
//				 changeX--;
//				 odometer.setX((changeX - 1) * TILE_SIZE);
//				 odometer.setTheta(0);
//				 Sound.beep();
//			 }
//			}
//			else {
//				
//			}


			correctionEnd = System.currentTimeMillis();

			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}

}