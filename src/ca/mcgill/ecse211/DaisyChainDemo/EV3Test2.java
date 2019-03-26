package ca.mcgill.ecse211.DaisyChainDemo;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.LED;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.remote.ev3.RemoteEV3;
import lejos.remote.ev3.RemoteRequestEV3;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class EV3Test2
{
	//Source code found on internet
	//Modified 
	private static Port t = LocalEV3.get().getPort("S1");

	public static void remoteLEDTest()
	{
		try {
			Brick first = BrickFinder.getLocal();

			System.out.println("Connect ans");
			//                BrickFinder.find(names[i])[0].getIPAddress()
			Brick second = new RemoteEV3("10.0.1.2");

			LED[] leds = new LED[2];
			leds[0] = first.getLED();
			leds[1] = second.getLED();
			int i = 0;
			int pat = 1;
			while(Button.ENTER.isUp())
			{
				leds[(i++) % leds.length].setPattern(0);
				if (i % leds.length == 0)
				{
					pat = ((pat + 1) % 3) + 1;
				}
				leds[(i) % leds.length].setPattern(pat);
				Delay.msDelay(100);
			}
			for(LED l : leds)
				l.setPattern(0);
		}
		catch (Exception e)
		{
			System.out.println("Got exception " + e);
		}
	}    

	public static void remoteMotorTest()
	{
		String[] names = {"what?", "ans"};
		RemoteRequestEV3[] bricks = new RemoteRequestEV3[2];
		try {
			for(int i = 1; i < names.length; i++)
				bricks[i] = new RemoteRequestEV3(BrickFinder.find(names[i])[0].getIPAddress());
			RegulatedMotor[] motors = new RegulatedMotor[bricks.length];
			motors[0] = new EV3LargeRegulatedMotor(BrickFinder.getLocal().getPort("A"));
			SensorModes touchSensor = new EV3TouchSensor(BrickFinder.getLocal().getPort("S1"));
			SampleProvider touch = touchSensor.getMode("Touch");
			//touch = bricks[0].createSampleProvider("S1", "EV3TouchSensor", "Touch")

					//   for(int i = 1; i < bricks.length; i++)
					//        {
					motors[1] = bricks[1].createRegulatedMotor("A", 'L');

			//      }

			for(RegulatedMotor m : motors)
			{
				m.setAcceleration(900);
				m.setSpeed(600);
			}
			boolean isMoving = false;
			while(true){
				float [] data = new float[touch.sampleSize()];
				touch.fetchSample(data, 0);
				System.out.println(data[0]);
				if(data[0] == 1 && isMoving == false) {
					motors[1].forward();
					Thread.sleep(1000);
					isMoving = true;
					
				}else if(data[0]==1&&isMoving == true) {
					motors[1].stop();
					Thread.sleep(1000);

					isMoving = false;
				}
			}
//
//			for(RegulatedMotor m : motors)
//				m.rotate(720);
//			for(RegulatedMotor m : motors)
//				m.rotate(-720);
//			for(RegulatedMotor m : motors)
//				m.rotate(720, true);
//			for(RegulatedMotor m : motors)
//				m.waitComplete();
//
//			for(RegulatedMotor m : motors)
//				m.rotate(-720, true);
//			for(RegulatedMotor m : motors)
//				m.waitComplete();
//
//			for(int i = 0; i < 4; i++)
//				for(RegulatedMotor m : motors)
//					m.rotate(90);
//			for(RegulatedMotor m : motors)
//				m.rotate(-720, true);
//			for(RegulatedMotor m : motors)
//				m.waitComplete();
//
//			for(int i = 0; i < motors.length; i++)
//			{
//				motors[i].setSpeed((i+1)*100);
//				motors[i].setAcceleration((i+1)*100);
//			}
//			for(int i = 0; i < motors.length; i++)
//				motors[i].rotate((i+1)*360, true);
//			for(RegulatedMotor m : motors)
//				m.waitComplete();
//
//			for(RegulatedMotor m : motors)
//				m.close();

			
		}
		catch (Exception e)
		{
			System.out.println("Got exception " + e);
		}
	}    
	public static void main(String[] args)
	{
		remoteMotorTest();
	}
}