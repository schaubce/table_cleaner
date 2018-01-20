package stupkajr.schaubce.cleartable;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class TestClass {

	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;
	
	public static void main(String[] args) {
		int speed = 360; // 1 RPM
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);		
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
		//EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
		while(true){
			SampleProvider distance;
			
			//sonicSensor.enable();
			//distance = sonicSensor.getDistanceMode();
			distance = colorSensor.getRedMode();
			float[] sample = new float[distance.sampleSize()];
			distance.fetchSample(sample, 0);
			System.out.println("RedMode is:");
			System.out.println(sample[0]);
			if(sample[0] > 0){	
				Sound.beep();
			}else if( sample[0] <= 0) {
				leftMotor.stop();
				rightMotor.stop();
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.stop();
				rightMotor.stop();
				Sound.beep();
				Sound.beep();
				System.exit(0);
				
			}
			leftMotor.forward();
			rightMotor.forward();
			
			if(Button.ENTER.isDown()){
				System.exit(0);
			}
		
		}
	}
	/*
	public static void main(String[] args) {
		Sound.beep();
		System.out.println("Running Test...");
		Delay.msDelay(2500);

		int speed = 360; // 1 RPM

		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);		
		leftMotor.forward();
		rightMotor.forward();
		Delay.msDelay(3000);
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.backward();
		rightMotor.backward();
		Delay.msDelay(3000);
		leftMotor.stop();
		rightMotor.stop();
		//1440 is equal to full 360 degree turn

		
		System.exit(0);

	}
	*/
}
