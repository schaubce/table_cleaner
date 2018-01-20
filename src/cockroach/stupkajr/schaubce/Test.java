package cockroach.stupkajr.schaubce;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Test {

	// Motors
	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;

	// Sensors
	static EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

	static int speed = 180; // 1 RPM
	static float currentAmbient =  0;


	 static SampleProvider sonicSample;
	 static SampleProvider gyroSample;
	 static SampleProvider lightSample;

	/******
	* Caitlin:" HEY JOHN HERE ARE SOME CODING STANDARDS FOR YOU TO REMEMBER:
	* 
	* 1. CamelCase 2. No Underscores 3. Variables/Methods should start
	* lowercase"
	* 
	* John: "Thanks! <3"
	* 
	*/

	public static void main(String[] args) {
	sonicSample = sonicSensor.getDistanceMode();
	gyroSample = gyroSensor.getAngleAndRateMode();
	lightSample = colorSensor.getAmbientMode();
	
	leftMotor.setSpeed(speed);
	rightMotor.setSpeed(speed);
	 
	leftMotor.forward();
	rightMotor.forward();
	
	
	
	float[] distance = new float[sonicSample.sampleSize()];
	sonicSample.fetchSample(distance, 0);
	float[] ambient = new float[lightSample.sampleSize()];
	lightSample.fetchSample(ambient, 0);
	
	currentAmbient = ambient[0];
	System.out.println(ambient[0]);
	
	// Check if we're at a wall
	while(atWall()){
		
		//fullSpin();
		
		// Check the ambient light, spin if heading into brighter area
		checkAmbient();
		
		if(Button.ENTER.isDown()){
			System.exit(0);
		}
	}
	

	}


	private static boolean atWall() {
		float[] distance = new float[sonicSample.sampleSize()];
		sonicSample.fetchSample(distance, 0);
		if(distance[0] <= 0.15){
			return false;
		}else{
			return true;	
		}		
	}
	
	private static void checkAmbient(){
		
		// Get ambient sample
		float[] ambient = new float[lightSample.sampleSize()];
		lightSample.fetchSample(ambient, 0);
		ambient = new float[lightSample.sampleSize()];
		lightSample.fetchSample(ambient, 0);
		
		
		// If the light we were in, was less than the light we're in now. Turn around
		if(currentAmbient < ambient[0]){
			ambient = new float[lightSample.sampleSize()];
			lightSample.fetchSample(ambient, 0);
			// Double check for hardware error
				if(currentAmbient < ambient[0]){
					
					Sound.beep();
					leftMotor.stop();
					rightMotor.stop();
					
					leftMotor.backward();
					rightMotor.forward();
					
					float[] angle = new float[gyroSample.sampleSize()];
					gyroSample.fetchSample(angle, 0);
					// Turn ~170 (Which is about 180 on carpet)
					while(Math.abs(angle[0]) < 170){
						Delay.msDelay(10);
						gyroSample.fetchSample(angle, 0);
					}
					
					leftMotor.stop();
					rightMotor.stop();
					Delay.msDelay(1000);
					// Reset the Gyro to zero and get the current ambient light
					currentAmbient = ambient[0];
					gyroSensor.reset();
					
					Delay.msDelay(1000);
					leftMotor.forward();
					rightMotor.forward();
			}
		}
				
	}
	
	private static void fullSpin(){
		
		float[] angle = new float[gyroSample.sampleSize()];
		gyroSample.fetchSample(angle, 0);
		
	}
	

}
