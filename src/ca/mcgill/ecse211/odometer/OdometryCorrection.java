package ca.mcgill.ecse211.odometer;

/*
 * OdometryCorrection.java
 */


import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
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
		
		odometer.setXYT(0, 0, 0);
		while (true) {
			correctionStart = System.currentTimeMillis();

			lightSensor.fetchSample(data, 0);
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