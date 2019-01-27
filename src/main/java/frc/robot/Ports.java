package frc.robot;

public class Ports {
		// TODO REMOVE UNUSED PORTS

		public static class Digital{
			public static final int CHECK_PRESSURE = 0;
			
		}
		
		public static class Analog{
			public static final int SONAR = 0;
			
		}
		
		public static class Relay{
			public static final int COMPRESSOR_RELAY = 0;
		}
		
		public static class CAN{
			/* 2017 robot
			public static final int RIGHT_REAR = 3;
			public static final int RIGHT_FRONT = 4;
			public static final int LEFT_REAR = 1;
			public static final int LEFT_FRONT = 2;
			public static final int SPIN = 5;
			public static final int CLIMB = 6;
			public static final int BASIN = 7;
			public static final int PCM = 8;
			public static final int PDP = 0;*/
			
			// 2018 robot
			public static final int RIGHT_FRONT = 1;
			public static final int RIGHT_REAR = 2;
			public static final int LEFT_FRONT = 3;
			public static final int LEFT_REAR = 4;
			public static final int FRONT_CENTER = 5;
			public static final int REAR_CENTER = 6;
			public static final int ELEVATOR = 7;
			public static final int GRASPER_LEFT = 8;
			public static final int GRASPER_RIGHT = 9;
			public static final int HINGE = 10;
			public static final int WINCH = 11; 
			public static final int PCM = 9;
			public static final int PDP = 0;
		}
		
		public static class USB{
			public static final int RIGHT = 0;
			public static final int LEFT = 1;
			public static final int GAMEPAD = 2;
		}
		
		public static class PCM{
			/* 2017 robot
			public static final int INTAKE_IN = 0;
			public static final int INTAKE_OUT = 1;
			public static final int INTAKE_DOWN = 2;
			public static final int INTAKE_UP = 3;
			public static final int GEAR_IN = 5;
			public static final int GEAR_OUT = 4;
			public static final int BASIN_DOWN = 6;
			public static final int BASIN_UP = 7;*/
			
			// 2018 robot
			public static final int JACK_DOWN = 0;
			public static final int JACK_UP = 1;
		}
}
