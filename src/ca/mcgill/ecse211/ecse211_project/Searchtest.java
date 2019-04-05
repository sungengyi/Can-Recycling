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
	public static final double TRACK = 17;//Width of wheel axis
	public static final double TILE_SIZE = 30.48;
	public static final double CAN_RADIUS = 5.5; // this is the offset from the 2 line-detecting light sensors to the wheel
	public static double HIT_GAP =0; //Radius of wheel
	public static boolean HEAVY = false;


	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port leftLightPort = LocalEV3.get().getPort("S2");
	private static final Port rightLightPort = LocalEV3.get().getPort("S1");
	private static final Port frontUSPort = LocalEV3.get().getPort("S3");
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor ultraMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	@SuppressWarnings("resource")
	public static final SensorModes leftLightSensor = new EV3ColorSensor(leftLightPort);//lightSensor is the instance 
	public static final SampleProvider colorData = leftLightSensor.getMode("RGB");

	@SuppressWarnings("resource") // Because we don't bother to close this resource
	public static SensorModes frontUSSensor = new EV3UltrasonicSensor(frontUSPort); // usSensor is the instance
	public static SampleProvider frontUSDistance = frontUSSensor.getMode("Distance");// usDistance provides samples from
	public static final EV3LargeRegulatedMotor upMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	public static void main(String [] args) throws IOException, OdometerExceptions {	
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
		Thread odoThread = new Thread(odometer);
		odoThread.start();	
		ColorData color = new ColorData(colorData);


		//-------------------Weighting Test------------------------------
		//---------------------------------------------------------------
		//The ultrasonic arm hits cans in front of it, push cans by a distance. 
		//Then it will print ultrasonic reading(distance) to the screen.
		//Find the best acceleration and speed that will diffrentiate heavy cans and light cans.
		//---------------------------------------------------------------
		//Runs in a loop. It's very slow to restart a program.
		//---------------------------------------------------------------
		//Comment this when not needed.
		//---------------------------------------------------------------
		//--------------------TESTING BODY-------------------------------
		//		for(int i = 0; i<=16;i++) {
		//			weightingTest();
		//			try {
		//				Thread.sleep(8000);
		//			} catch (InterruptedException e) {
		//
		//			}
		//		}
		//------------------TESTING END-----------------------------------
		//----------------------------------------------------------------



		//-------------------Color Identify Test------------------------------
		//---------------------------------------------------------------
		//The color arm is initially placed on the left.
		//Open the ultrasonic arm and place cans.
		//---------------------------------------------------------------
		//---------------------------------------------------------------
		//Comment this when not needed.
		//Runs in a thread.
		//---------------------------------------------------------------
		//--------------------TESTING BODY-------------------------------
//		while(true) {
//			color.identify();
//			try {
//				Thread.sleep(3000);
//			} catch (InterruptedException e) {
//			}
//		}
		//------------------TESTING END
		//----------------------------------------------------------------


		//-------------------Searching Test------------------------------
		//---------------------------------------------------------------
		//The robot spins clockwise and detect for any can in its range.
		//When a can is detected, robot moves by half of the distance the can was detected.
		//It will restart scanning for cans and will resursively do the above actions until the can is very near.
		//---------------------------------------------------------------
		//---------------------------------------------------------------
		//Comment this when not needed.
		//---------------------------------------------------------------
		//--------------------TESTING BODY-------------------------------
		SearchAndGrabTest();
		//------------------TESTING END
		//----------------------------------------------------------------


	}
	public static void SearchAndGrabTest() {
		ColorData color = new ColorData(colorData);
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.setAcceleration(10000);
		rightMotor.setAcceleration(10000);
		upMotor.setSpeed(200);

		//find the can and stop in front of it
		detectForCans(43);
		do {
			leftMotor.forward();
			rightMotor.forward();
		}while(getUSData(frontUSSensor)<10000);

		leftMotor.stop(true);
		rightMotor.stop();

		weightingTest();
		ultraMotor.setSpeed(100);
		//go back for claw to grab
		leftMotor.rotate((convertDistance(WHEEL_RAD,-CAN_RADIUS)),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,-CAN_RADIUS)),false);

		//open
		ultraMotor.rotate(100);

		//go forward
		leftMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + 2*CAN_RADIUS))),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + 2*CAN_RADIUS))),false);

		//identify
	//	color.identify();
		
		leftMotor.rotate((convertDistance(WHEEL_RAD,CAN_RADIUS)),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,CAN_RADIUS)),false);
		//grab
		ultraMotor.rotate(-100);
		
		leftMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + CAN_RADIUS))),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + CAN_RADIUS))),false);
		


	}
	public static void weightingTest() {
		ultraMotor.setSpeed(1000);
		ultraMotor.setAcceleration(15000);
		ultraMotor.rotate(20);
		ultraMotor.rotate(-20);
		HIT_GAP = getUSData(frontUSSensor);
		if(HIT_GAP < 7)  HEAVY=true;
		else if(HIT_GAP > 15) HIT_GAP = 15;
			
		System.out.println("The can is "+HIT_GAP+"away.");
	}
	public static void detectForCans(double range) {
		leftMotor.forward();
		rightMotor.backward();
		long start, end;
		double period = 100;
		double distance;
		boolean detected = false;
		boolean switchs = true;
		int counter = 0;
		while(range > 5 && !detected) {
			
			start = System.currentTimeMillis();
			distance = getUSData(frontUSDistance);
			if(distance>range) {
				if(switchs) {
					leftMotor.forward();
					rightMotor.backward();
					counter++;
					if(counter%20==0) switchs = !switchs;
					
				}else {
				
				leftMotor.backward();
				rightMotor.forward();
				counter++;
				if(counter%20==0) switchs = !switchs;
			
				}
			}
			if(distance < 5) {
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
