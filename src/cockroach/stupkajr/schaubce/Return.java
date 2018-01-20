package cockroach.stupkajr.schaubce;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Return {

	// Motors
		static RegulatedMotor leftMotor = Motor.D;
		static RegulatedMotor rightMotor = Motor.A;

		// Sensors
		static EV3UltrasonicSensor sonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
		static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);
		static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

		 static int speed = 720; // 1 RPM


		 static SampleProvider sonicSample;
		 static SampleProvider gyroSample;

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
				
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
		 
			leftMotor.backward();
			rightMotor.backward();
				
			Delay.msDelay(2500);
				
		}
	
}
