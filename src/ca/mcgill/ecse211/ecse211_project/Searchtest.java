package ca.mcgill.ecse211.ecse211_project;

import java.io.IOException;

import ca.mcgill.ecse211.color.ColorData;
import ca.mcgill.ecse211.navigator.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.remote.ev3.RemoteRequestEV3;
import lejos.robotics.ColorIdentifier;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class Searchtest {
	public static final double WHEEL_RAD = 2.1; //Radius of wheel
	public static final double TRACK = 13.6;//Width of wheel axis
	public static final double TILE_SIZE = 30.48;
	public static final double OFF_SET = 2.5; // this is the offset from the 2 line-detecting light sensors to the wheel

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port leftLightPort = LocalEV3.get().getPort("S2");
	private static final Port rightLightPort = LocalEV3.get().getPort("S4");
	private static final Port frontUSPort = LocalEV3.get().getPort("S3");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes frontUSSensor = new EV3UltrasonicSensor(frontUSPort); // usSensor is the instance
	public static SampleProvider frontUSDistance = frontUSSensor.getMode("Distance");// usDistance provides samples from
	public static final EV3LargeRegulatedMotor upMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	public static void main(String [] args) throws IOException, OdometerExceptions {	
		@SuppressWarnings("resource")
		SensorModes leftLightSensor = new EV3ColorSensor(leftLightPort);//lightSensor is the instance 
		SampleProvider colorData = leftLightSensor.getMode("RGB");
//		RemoteRequestEV3 slaveBrick = new RemoteRequestEV3("10.0.1.2");
//		SampleProvider colorData = slaveBrick.createSampleProvider("S1", "EV3ColorSensor", "RGB");
//		RegulatedMotor upMotor = slaveBrick.createRegulatedMotor("A", 'L');
		ColorData color = new ColorData(colorData);
		double[][] x = new double [4][4];
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Navigator nimbus = new Navigator(x, odometer,leftMotor, rightMotor, TRACK,WHEEL_RAD);
		leftMotor.setSpeed(250);
		rightMotor.setSpeed(250);
		leftMotor.forward();
		rightMotor.backward();
		int i =0;
		while(true) {
			detectForCans(43);
			color.identify();
			nimbus.turnTo(nimbus.getDesAngle(TILE_SIZE+i,0));
			nimbus.travelTo(TILE_SIZE+i, 0);
			nimbus.turnTo(0);
			if(i==5) break;
			i++;
		}
	}
	public static void detectForCans(double range) {
		leftMotor.forward();
		rightMotor.backward();
		long start, end;
		double period = 100;
		double distance;
		boolean detected = false;
		while(range > 5 || !detected) {
			start = System.currentTimeMillis();
			distance = getUSData(frontUSDistance);
			System.out.println(distance);
			if(distance < 3) {
				leftMotor.rotate(convertDistance(WHEEL_RAD, distance/2), true);
				rightMotor.rotate(convertDistance(WHEEL_RAD, distance/2), false);
				leftMotor.stop(true);
				rightMotor.stop();
				Sound.beep();
				detected = true;
			}
			if(distance <= range) {
				leftMotor.stop(true);
				rightMotor.stop();
				leftMotor.rotate(convertDistance(WHEEL_RAD, distance/2), true);
				rightMotor.rotate(convertDistance(WHEEL_RAD, distance/2), false);
				range /= 2;
			}
			end = System.currentTimeMillis();
			if((end - start)< period) {
				try {
					Thread.sleep(30);
				} catch (InterruptedException e) {
				}
			}
		}



	}
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * This method comes from SquareDriver class. It converts the desired turning
	 * angle to the degrees of rotation of the motor.
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	private static int getUSData(SampleProvider usDistance) {	
		float[] usData = new float [usDistance.sampleSize()];
		usDistance.fetchSample(usData, 0); // acquire data
		int distance =  (int) (usData[0] * 100.0); // extract from buffer, cast to int
		return distance;
	}

}
