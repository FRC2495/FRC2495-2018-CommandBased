/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.interfaces.*;
import frc.robot.sensors.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.*;
import frc.robot.interfaces.IJack.Position;;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	// IMPORTANT MAKE SURE THAT THIS CONSTANT IS SET TO TRUE IF USING COMPETITION BOT!
	// use this constant to switch between competition and practice bot
	public static final boolean COMPETITION_BOT_CONFIG = true;

	//public static OI oi;

	Command m_autonomousCommand;
	
	Command gyroCalibrateAndReset;
	Command gyroReset;
	
	Command drivetrainJoystickControl;
	Command drivetrainSingleJoystickControl;
	Command drivetrainMoveDistance50inches;
	Command drivetrainTurnAngleUsingPidControllerPlus90degrees;
	Command drivetrainTurnAngleUsingPidControllerMinus90degrees;
	Command drivetrainStop;
	Command drivetrainResetEncoders;

	Command miniDrivetrainJoystickControl;
	Command miniDrivetrainMoveDistance50inches;
	Command miniDrivetrainMoveUsingCameraPidController;
	Command miniDrivetrainStop;
	Command miniDrivetrainResetEncoders;

	Command hingeJoystickControl;
	Command hingeHome;
	Command hingeMoveDown;
	Command hingeMoveMidway;
	Command hingeMoveUp;
	Command hingeStop;

	Command elevatorJoystickControl;
	Command elevatorHome;
	Command elevatorMoveDown;
	Command elevatorMoveMidway;
	Command elevatorMoveUp;
	Command elevatorStop;
	
	Command jackSetPositionLargeDrivetrain;
	Command jackSetPositionMiniDrivetrain;

	Command grasperJoystickControl;
	Command grasperGrasp;
	Command grasperRelease;
	Command grasperStop;
	
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	// Change this constant to choose between one or two joysticks for main drivetrain
	public static /*final*/ boolean USE_TWO_JOYSTICKS_TO_DRIVE = true;
	
	// set the following two constants to true if using a proto/second bot with no hinge and/or no elevator
	public static final boolean HINGE_DISABLED = false;
	public static final boolean ELEVATOR_DISABLED = false;	
	
	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_CUSTOM = "My Auto";
	private String autonSelected;
	private SendableChooser<String> autonChooser = new SendableChooser<>();
	
	public static final String START_POSITION_LEFT = "Left";
	public static final String START_POSITION_CENTER = "Center";
	public static final String START_POSITION_RIGHT = "Right";
	private String startPosition;
	private SendableChooser<String> startPositionChooser = new SendableChooser<>();
	
	public static final String CAMERA_OPTION_USE_ALWAYS = "Always";
	public static final String CAMERA_OPTION_USE_OPEN_LOOP_ONLY = "Open Loop Only";
	public static final String CAMERA_OPTION_USE_CLOSED_LOOP_ONLY = "Closed Loop Only";
	public static final String CAMERA_OPTION_USE_NEVER = "Never";
	private String cameraOption;
	private SendableChooser<String> cameraOptionChooser = new SendableChooser<>();
	
	public static final String SONAR_OPTION_USE_ALWAYS = "Always";
	public static final String SONAR_OPTION_USE_RELEASE_ONLY = "Release Only";
	public static final String SONAR_OPTION_USE_GRASP_ONLY = "Grasp Only";
	public static final String SONAR_OPTION_USE_NEVER = "Never";
	private String sonarOption;
	private SendableChooser<String> sonarOptionChooser = new SendableChooser<>();
	
	public static final String GRASPER_OPTION_RELEASE = "Release";
	public static final String GRASPER_OPTION_DONT_RELEASE = "Don't Release"; 
	private String releaseSelected;
	private SendableChooser<String> releaseChooser = new SendableChooser<>();


	// sensors
	
	HMCamera camera;
	
	public static ADXRS450_Gyro gyro; // gyro
	boolean hasGyroBeenManuallyCalibratedAtLeastOnce = false;
	
	Sonar sonar;
	
	HMAccelerometer accelerometer;
	
	// motorized devices
	
	public static /*I*/Drivetrain drivetrain;

	WPI_TalonSRX frontLeft;
	WPI_TalonSRX frontRight;
	BaseMotorController rearLeft; 
	BaseMotorController rearRight;
	
	public static /*I*/MiniDrivetrain miniDrivetrain;
	
	WPI_TalonSRX frontCenter;
	WPI_TalonSRX rearCenter;

	public static boolean elevatorFlagUp = true;
	public static /*I*/Elevator elevatorControl;
	
	WPI_TalonSRX elevator;
	
	public static /*I*/Grasper grasper;
	
	BaseMotorController grasperLeft;
	BaseMotorController grasperRight;
	
	public static boolean hingeFlagUp = false;
	public static /*I*/Hinge hingeControl;
	
	WPI_TalonSRX hinge; 
	
	//IWinch winchControl;
	
	//BaseMotorController winch;
	
	// pneumatic devices
	
	Compressor compressor; // the compressor's lifecycle needs to be the same as the robot
	
	public static /*I*/Jack jack;
	public static boolean largeDriveTrainSelected = false; // by default we assume small drivetrain is down
	
	// joysticks and gamepad
	
	public static ControllerBase control;
	
	public static Joystick joyLeft, joyRight;
	public static Joystick gamepad;
	
	//misc. 
	
	GameData gameData;
	
	//Auton auton = null; // autonomous stuff
	
	PositionTracker tracker;

  	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
    	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=
		// choosers (for auton)
		
		autonChooser.setDefaultOption("Do Nothing", AUTON_DO_NOTHING);
		autonChooser.addOption("My Auto", AUTON_CUSTOM);
		SmartDashboard.putData("Auto choices", autonChooser);
		
		startPositionChooser.setDefaultOption("Left", START_POSITION_LEFT);
		startPositionChooser.addOption("Center", START_POSITION_CENTER);
		startPositionChooser.addOption("Right", START_POSITION_RIGHT);
		SmartDashboard.putData("Start positions", startPositionChooser);
		
		cameraOptionChooser.addOption("Always", CAMERA_OPTION_USE_ALWAYS);
		cameraOptionChooser.setDefaultOption("Open Loop Only", CAMERA_OPTION_USE_OPEN_LOOP_ONLY);
		cameraOptionChooser.addOption("Closed Loop Only", CAMERA_OPTION_USE_CLOSED_LOOP_ONLY);
		cameraOptionChooser.addOption("Never", CAMERA_OPTION_USE_NEVER);		
		SmartDashboard.putData("Camera options", cameraOptionChooser);
		
		sonarOptionChooser.setDefaultOption("Always", SONAR_OPTION_USE_ALWAYS);
		sonarOptionChooser.addOption("Release Only", SONAR_OPTION_USE_RELEASE_ONLY);
		sonarOptionChooser.addOption("Grasp Only", SONAR_OPTION_USE_GRASP_ONLY);		
		sonarOptionChooser.addOption("Never", SONAR_OPTION_USE_NEVER);
		SmartDashboard.putData("Sonar options", sonarOptionChooser);
		
		releaseChooser.setDefaultOption("Release", GRASPER_OPTION_RELEASE);
		releaseChooser.addOption("Don't release", GRASPER_OPTION_DONT_RELEASE);
		SmartDashboard.putData("Release options", releaseChooser);
    

		// sensors
			
		sonar = new Sonar(Ports.Analog.SONAR); 
			
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); // we want to instantiate before we pass to drivetrain	

		gyro.calibrate(); 
		gyro.reset();

		camera = new HMCamera("GRIP/myContoursReport");

		accelerometer = new HMAccelerometer();

		// motorized devices

		if (COMPETITION_BOT_CONFIG) {
			frontLeft = new WPI_TalonSRX(Ports.CAN.LEFT_FRONT);
			frontRight = new WPI_TalonSRX(Ports.CAN.RIGHT_FRONT);
			rearLeft = new WPI_TalonSRX(Ports.CAN.LEFT_REAR);
			rearRight= new WPI_TalonSRX(Ports.CAN.RIGHT_REAR);	
		} else {
			frontLeft = new WPI_TalonSRX(Ports.CAN.LEFT_FRONT);
			frontRight = new WPI_TalonSRX(Ports.CAN.RIGHT_FRONT);
			rearLeft = new WPI_VictorSPX(Ports.CAN.LEFT_REAR);
			rearRight= new WPI_VictorSPX(Ports.CAN.RIGHT_REAR);
		}

		frontCenter= new WPI_TalonSRX(Ports.CAN.FRONT_CENTER);
		rearCenter= new WPI_TalonSRX(Ports.CAN.REAR_CENTER);

		elevator = new WPI_TalonSRX(Ports.CAN.ELEVATOR);

		if (COMPETITION_BOT_CONFIG) {
			grasperLeft = new WPI_TalonSRX(Ports.CAN.GRASPER_LEFT);
			grasperRight = new WPI_TalonSRX(Ports.CAN.GRASPER_RIGHT);
		} else {
			grasperLeft = new WPI_VictorSPX(Ports.CAN.GRASPER_LEFT);
			grasperRight = new WPI_VictorSPX(Ports.CAN.GRASPER_RIGHT);
		}

		hinge = new WPI_TalonSRX(Ports.CAN.HINGE);

		//winch = new WPI_TalonSRX(Ports.CAN.WINCH);


		tracker = new PositionTracker();


		if (COMPETITION_BOT_CONFIG) {
			jack = new Jack();
			//jack = new EmulatedJack();
			
			drivetrain = new Drivetrain( frontLeft, frontRight, rearLeft, rearRight, gyro, this);	
			//drivetrain = new EmulatedDrivetrain(jack, tracker);	
			
			miniDrivetrain = new MiniDrivetrain(frontCenter, rearCenter, gyro, this, camera);
			//miniDrivetrain = new EmulatedMiniDrivetrain(jack, tracker);
			
			hingeControl = new Hinge(hinge, this);
			//hingeControl = new EmulatedHinge();		
			
			elevatorControl = new Elevator(elevator, hingeControl, this);
			//elevatorControl = new EmulatedElevator(hingeControl, tracker);
			
			grasper = new Grasper(grasperLeft, grasperRight, sonar, this);
			//grasper = new EmulatedGrasper(hingeControl, elevatorControl);
			
			//winchControl = new Winch(winch, this);
			//winchControl = new EmulatedWinch();
		} else {
			jack = new Jack();
			//jack = new EmulatedJack();
			
			drivetrain = new Drivetrain( frontLeft, frontRight, rearLeft, rearRight, gyro, this);	
			//drivetrain = new EmulatedDrivetrain(jack, tracker);	
			
			miniDrivetrain = new MiniDrivetrain(frontCenter, rearCenter, gyro, this, camera);
			//miniDrivetrain = new EmulatedMiniDrivetrain(jack, tracker);
			
			hingeControl = new Hinge(hinge, this);
			//hingeControl = new EmulatedHinge();		
			
			elevatorControl = new Elevator(elevator, hingeControl, this);
			//elevatorControl = new EmulatedElevator(hingeControl, tracker);
			
			grasper = new Grasper(grasperLeft, grasperRight, sonar, this);
			//grasper = new EmulatedGrasper(hingeControl, elevatorControl);
			
			//winchControl = new Winch(winch, this);
			//winchControl = new EmulatedWinch();
		}

		// pneumatic devices

		compressor = new Compressor();
		compressor.checkCompressor();

		// joysticks and gamepad

		joyLeft = new Joystick(Ports.USB.LEFT); 
		joyRight = new Joystick(Ports.USB.RIGHT);

		gamepad = new Joystick(Ports.USB.GAMEPAD);

		control = new ControllerBase(gamepad, joyLeft, joyRight);	

		// misc.

		gameData = new GameData();
		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
		
		// OI must be constructed after subsystems. If the OI creates Commands
		//(which it very likely will), subsystems are not guaranteed to be
		// constructed yet. Thus, their requires() statements may grab null
		// pointers. Bad news. Don't move it.
		//oi = new OI();

		// commands

		gyroCalibrateAndReset = new GyroCalibrateAndReset();
		gyroReset = new GyroReset();

		drivetrainJoystickControl = new DrivetrainJoystickControl();
		drivetrainSingleJoystickControl = new DrivetrainSingleJoystickControl();
		drivetrainMoveDistance50inches = new DrivetrainMoveDistance(50);
		drivetrainTurnAngleUsingPidControllerPlus90degrees = new DrivetrainTurnAngleUsingPidController(90);
		drivetrainTurnAngleUsingPidControllerMinus90degrees = new DrivetrainTurnAngleUsingPidController(-90);
		drivetrainStop = new DrivetrainStop();
		drivetrainResetEncoders = new DrivetrainResetEncoders();
		
		miniDrivetrainJoystickControl = new MiniDrivetrainJoystickControl();
		miniDrivetrainMoveDistance50inches = new MiniDrivetrainMoveDistance(50);
		miniDrivetrainMoveUsingCameraPidController = new MiniDrivetrainMoveUsingCameraPidController();
		miniDrivetrainStop = new MiniDrivetrainStop();
		miniDrivetrainResetEncoders = new MiniDrivetrainResetEncoders();
		
		hingeJoystickControl = new HingeJoystickControl();
		hingeHome = new HingeHome();
		hingeMoveDown = new HingeMoveDown();
		hingeMoveMidway = new HingeMoveMidway();
		hingeMoveUp = new HingeMoveUp();
		hingeStop = new HingeStop();
	
		elevatorJoystickControl = new ElevatorJoystickControl();
		elevatorHome = new ElevatorHome();
		elevatorMoveDown = new ElevatorMoveDown();
		elevatorMoveMidway = new ElevatorMoveMidway();
		elevatorMoveUp = new ElevatorMoveUp();
		elevatorStop = new ElevatorStop();
	
		jackSetPositionMiniDrivetrain = new JackSetPosition(IJack.Position.MINI_DRIVETRAIN);
		jackSetPositionLargeDrivetrain = new JackSetPosition(IJack.Position.LARGE_DRIVETRAIN);

		grasperJoystickControl = new GrasperJoystickControl();
		grasperGrasp = new GrasperGrasp();
		grasperRelease = new GrasperRelease();
		grasperStop = new GrasperStop();
	
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use
	 * this for items like diagnostics that you want ran during disabled,
	 * autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
	}
  
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);
		
		startPosition = startPositionChooser.getSelected();
		System.out.println("Start position: " + startPosition);
		
		cameraOption = cameraOptionChooser.getSelected();
		System.out.println("Camera option: " + cameraOption);
		
		sonarOption = sonarOptionChooser.getSelected();
		System.out.println("Sonar option: " + sonarOption);
		
		releaseSelected = releaseChooser.getSelected();
		System.out.println("Release chosen: " + releaseSelected);
		
		gameData.update();
		
		//At this point we should know what auto run, where we started, and where our plates are located.
		//So we are ready for autonomousPeriodic to be called.
		updateToSmartDash();

		m_autonomousCommand = null; // TODO decide which command to run based on user's selection


		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
  }

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		
		gameData.update();	
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		control.update();
		camera.acquireTargets(false);
		
		Scheduler.getInstance().run(); // forces execution of next scheduled command (from prior iteration)
    

   	//RIGHT JOYSTICK //RIGHT JOYSTICK //RIGHT JOYSTICK //RIGHT JOYSTICK //RIGHT JOYSTICK
		
		//Joystick drive only using right joystick
		if (largeDriveTrainSelected) {
			if (USE_TWO_JOYSTICKS_TO_DRIVE) {
				
				if (!control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN8)
					&& !control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN9)
					&& !control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN10)
					&& !control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN11)) {
				
					if (!drivetrain.isMoving() && !drivetrain.isTurning() && !drivetrainJoystickControl.isRunning()) {  
						drivetrainJoystickControl.start();
					}
				}
			} else {
				if (!drivetrain.isMoving() && !drivetrain.isTurning() && !drivetrainSingleJoystickControl.isRunning()) {  
					drivetrainSingleJoystickControl.start();
				}
			}
		}
		else
		{
			if (!miniDrivetrain.isMoving() && !miniDrivetrain.isMovingUsingCamera() && !miniDrivetrainJoystickControl.isRunning()) {  
				miniDrivetrainJoystickControl.start();
			}
		}			
		
		// Jack up or down the robot to switch between main or mini drivetrain			
		if (control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK,ControllerBase.JoystickButtons.BTN2))
		{
			System.out.println("Button RIGHT.BTN2 Pushed");
			
			largeDriveTrainSelected = false; 
			System.out.println("jack down");
			jackSetPositionMiniDrivetrain.start();
			drivetrainStop.start();
		}
				
		if (control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK,ControllerBase.JoystickButtons.BTN3))
		{
			System.out.println("Button RIGHT.BTN3 Pushed");
					
			largeDriveTrainSelected = true; 
			System.out.println("jack up");
			jackSetPositionLargeDrivetrain.start();
			miniDrivetrainStop.start();
		} 
    
   		//Resets encoders (and gyro) "Reset"
		if (control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN6))
		{
			System.out.println("Button RIGHT.BTN6 Pushed");
			
			drivetrainResetEncoders.start();
			miniDrivetrainResetEncoders.start();;
			
			//gyroReset.start(); // resets to zero - we don't want to rirsk loosing time during competition
		}		
		
		//Stops the robot moving if pressed (or any closed loop operation) "Abort"
		if (control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN7))
		{
			System.out.println("Button RIGHT.BTN7 Pushed");
			
			drivetrainStop.start();
			miniDrivetrainStop.start();
		}
		
		if (control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN8))
		{
			System.out.println("Button RIGHT.BTN8 Pushed");
			
			System.out.println("Switching forward control to left joystick.");
			
			USE_TWO_JOYSTICKS_TO_DRIVE = true;
		}
		
		if (control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN9))
		{
			System.out.println("Button RIGHT.BTN9 Pushed");
			
			System.out.println("Switching forward control to right joystick.");
			
			USE_TWO_JOYSTICKS_TO_DRIVE = false;
		}
 

		// LEFT JOYSTICK // LEFT JOYSTICK // LEFT JOYSTICK // LEFT JOYSTICK // LEFT JOYSTICK
						
		// todo document new option
		if (control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN2))
		{
			System.out.println("Button LEFT.BTN2 Pushed");
			
			if (jack != null && (jack.getPosition() != Position.MINI_DRIVETRAIN)) {
				System.out.println("WARNING: moving mini drivetrain when jack is not down!");
			}
			
			miniDrivetrainMoveUsingCameraPidController.start();
		}
		else if (control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN3))
		{
			System.out.println("Button LEFT.BTN3 Pushed");	
			
			if (largeDriveTrainSelected) {
				drivetrainMoveDistance50inches.start();
			} else {
				miniDrivetrainMoveDistance50inches.start();
			}
		}
		else if (control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN4))
		{
			System.out.println("Button LEFT.BTN4 Pushed");
			
			if (jack != null && (jack.getPosition() != Position.LARGE_DRIVETRAIN)) {
				System.out.println("WARNING: turning drivetrain when jack is not up!");
			}
			
			//drivetrain.moveDistanceAlongArc(-90);
			drivetrainTurnAngleUsingPidControllerMinus90degrees.start();
		}
		else if (control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN5))
		{
			System.out.println("Button LEFT.BTN5 Pushed");
			
			if (jack != null && (jack.getPosition() != Position.LARGE_DRIVETRAIN)) {
				System.out.println("WARNING: turning drivetrain when jack is not up!");
			}
			
			//drivetrain.moveDistanceAlongArc(+90);
			drivetrainTurnAngleUsingPidControllerPlus90degrees.start();
		}
		
		//Resets encoders (and gyro) "Reset"
		if (control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN6))
		{
			System.out.println("Button LEFT.BTN6 Pushed");
			
			drivetrainResetEncoders.start();
			miniDrivetrainResetEncoders.start();
			
			gyroReset.start();
		}		
		
		//Stops the robot moving if pressed (or any closed loop operation) "Abort"
		if (control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN7))
		{
			System.out.println("Button LEFT.BTN7 Pushed");
			
			drivetrainStop.start();
			miniDrivetrainStop.start();
		}
				
		if (control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN8))
		{
			if (!elevatorControl.isMoving() && !elevatorControl.isHoming() && !elevatorJoystickControl.isRunning()) {  
				elevatorJoystickControl.start();
			}
		}
		
		if (control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN9))
		{	
			if (!hingeControl.isMoving() && !hingeControl.isHoming() && !hingeJoystickControl.isRunning()) {  
				hingeJoystickControl.start();
			}
		}
		
		if (control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN10))
		{
			if (!grasper.isGrasping() && !grasper.isReleasing() && !grasperJoystickControl.isRunning()) {  
				grasperJoystickControl.start();
			}
		}	
		
		// auto-stop if we release one of the buttons before releasing joystick
		if (control.getReleased(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN8))
		{
			elevatorStop.start();
		}
		
		if (control.getReleased(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN9))
		{	
			hingeStop.start();
		}
		
		if (control.getReleased(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN10))
		{
			grasperStop.start();
		}

		
		// GAMEPAD // GAMEPAD // GAMEPAD // GAMEPAD // GAMEPAD

		//Home Elevator
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.BACK)) {
			System.out.println("Button BACK Pushed");
			
			if (hingeControl != null && (!hingeControl.hasBeenHomed() || !hingeControl.isDown())) {
				System.out.println("ERROR: cannot home elevator up when hinge has not been homed or is not down!");
			} else {
				elevatorHome.start();
			}
		}
		
		//Home Hinge
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.START)) { 
			System.out.println("Button START Pushed");
			
			hingeHome.start();
		}
		
		//elevator up and down using Left LS button
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.LS)) {
			System.out.println("Button LS Pushed");
			
			if (hingeControl != null && (!hingeControl.hasBeenHomed() || !hingeControl.isDown())) {
				System.out.println("ERROR: cannot move elevator up or down when hinge has not been homed or is not down!");
			} else {
				if (elevatorFlagUp) {
					elevatorMoveUp.start();
					System.out.println("Elevator should be moving up");
					elevatorFlagUp = false;
				} else {
					elevatorMoveDown.start();
					System.out.println("Elevator should be moving down");
					elevatorFlagUp = true;
				}
			}
		}
		
		//hinge up and down using Right RS button
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.RS)) {
			System.out.println("Button RS Pushed");

			if (hingeFlagUp) {
				hingeMoveUp.start();
				System.out.println("Hinge should be moving up");
				hingeFlagUp = false;
			} else {
				hingeMoveDown.start();
				System.out.println("Hinge should be moving down");
				hingeFlagUp = true;
			}
		}
		
		//Use Left bumper to move elevator midway (switch)
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.LB)) { 
			System.out.println("Button LB Pushed");
			
			if (hingeControl != null && (!hingeControl.hasBeenHomed() || !hingeControl.isDown())) {
				System.out.println("ERROR: cannot move elevator midway when hinge has not been homed or is not down!");
			} else {
				elevatorMoveMidway.start();
				System.out.println("Elevator should be moving midway");
				elevatorFlagUp = false;
			}
		}
		
		//Use Right bumper to move hinge midway (throw)
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.RB)) { 
			System.out.println("Button RB Pushed");
			
			hingeMoveMidway.start();
			System.out.println("Hinge should be moving midway");
			hingeFlagUp = false;
		}

		//abort bound to X
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.X)) {
			System.out.println("Button X Pushed");
			
			//drivetrainStop.start();
			//miniDrivetrainStop.start();
			
			hingeStop.start();
			elevatorStop.start();
			grasperStop.start();
		}
		
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.A)) { 
			System.out.println("Button A Pushed");
			
			if (hingeControl != null && (!hingeControl.hasBeenHomed() || !hingeControl.isDown())) {
				System.out.println("WARNING: grasping when hinge has not been homed or is not down!");
			}
			
			if (elevatorControl != null && (!elevatorControl.hasBeenHomed() || !elevatorControl.isDown())) {
				System.out.println("WARNING: grasping when elevator has not been homed or is not down!");
			}
			
			grasperGrasp.start();
		}
		else if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.B)) {
			System.out.println("Button B Pushed");
			
			if (hingeControl != null && (!hingeControl.hasBeenHomed() || hingeControl.isUp())) {
				System.out.println("WARNING: releasing when hinge has not been homed or is up!");
			}
			
			if (elevatorControl != null && (!elevatorControl.hasBeenHomed() || elevatorControl.isDown())) {
				System.out.println("WARNING: releasing when elevator has not been homed or is down!");
			}
			
			grasperRelease.start();
		}
		else 
		{
			//grasperStop.start();	// for manual mode, remove if auto stop is desired	
		}
		
		// experimental code to see if we can detect gamepad axes virtually pressed as buttons
		if (control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.LX, true) ||
				control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.LY, true) || 
				control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.LX, false) ||
				control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.LY, false)) {
			
			System.out.println("Gamepad axis L Pushed");
			
			if (hingeControl != null && (!hingeControl.hasBeenHomed() || !hingeControl.isDown())) {
				System.out.println("ERROR: cannot move elevator up or down when hinge has not been homed or is not down!");
			} else {
				elevatorMoveUp.start();
				System.out.println("Elevator should be moving up");
				elevatorFlagUp = false;
			}
		}
		
		if (control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.LT, true)) {
			System.out.println("Gamepad axis LT Pushed positively");
			
			if (hingeControl != null && (!hingeControl.hasBeenHomed() || !hingeControl.isDown())) {
				System.out.println("ERROR: cannot move elevator up or down when hinge has not been homed or is not down!");
			} else {
				elevatorMoveDown.start();
				System.out.println("Elevator should be moving down");
				elevatorFlagUp = true;
			}
		}
		
		if (control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.RT, true)) {
			System.out.println("Gamepad axis RT Pushed positively");
			
			hingeMoveDown.start();
			System.out.println("Hinge should be moving down");
			hingeFlagUp = true;
		}
		
		if (control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.RX, true) ||
				control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.RY, true) ||
				control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.RX, false) ||
				control.getGamepadAxisPressedDown(ControllerBase.GamepadAxes.RY, false)) {
			
			System.out.println("Gamepad axis R Pushed");
			
			hingeMoveUp.start();
			System.out.println("Hinge should be moving up");
			hingeFlagUp = false;
		}

    
    Scheduler.getInstance().run(); // schedule command(s) that were added in this iteration

		camera.acquireTargets(false);
		updateToSmartDash(); 	
  }

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		gameData.update();
	}

	@Override
	public void disabledPeriodic() {
		control.update();
		camera.acquireTargets(false);

		Scheduler.getInstance().run(); // forces execution of next scheduled command (from prior iteration)

		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.A)) {
      gyroCalibrateAndReset.start();
			hasGyroBeenManuallyCalibratedAtLeastOnce = true; // we flag that this was done
		}

	  Scheduler.getInstance().run(); // schedule command(s) that were added in this iteration

    camera.acquireTargets(false);
		updateToSmartDash();    
  }

	public void updateToSmartDash()
	{
		// Send Gyro val to Dashboard
		SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
			
		SmartDashboard.putNumber("Right Value", drivetrain.getRightPosition());
		SmartDashboard.putNumber("Left Value", drivetrain.getLeftPosition());
		SmartDashboard.putNumber("Right Enc Value", drivetrain.getRightEncoderPosition());
		SmartDashboard.putNumber("Left Enc Value", drivetrain.getLeftEncoderPosition());
		SmartDashboard.putNumber("Right Enc Velocity", drivetrain.getRightEncoderVelocity());
		SmartDashboard.putNumber("Left Enc Velocity", drivetrain.getLeftEncoderVelocity());
		SmartDashboard.putBoolean("isMoving?", drivetrain.isMoving());
		SmartDashboard.putBoolean("isTurning?", drivetrain.isTurning());
		SmartDashboard.putBoolean("isStalled?", drivetrain.isStalled());
		
		SmartDashboard.putNumber("Mini Right Value", miniDrivetrain.getRightPosition());
		SmartDashboard.putNumber("Mini Left Value", miniDrivetrain.getLeftPosition());
		SmartDashboard.putNumber("Mini Right Enc Value", miniDrivetrain.getRightEncoderPosition());
		SmartDashboard.putNumber("Mini Left Enc Value", miniDrivetrain.getLeftEncoderPosition());
		SmartDashboard.putBoolean("Mini isMoving?", miniDrivetrain.isMoving());
		
		SmartDashboard.putBoolean("isCompromised?", DriverStation.getInstance().isDisabled());
		
		SmartDashboard.putNumber("Distance to Target", camera.getDistanceToTargetUsingVerticalFov());
		SmartDashboard.putNumber("Angle to Target", camera.getAngleToTurnToTarget());
		SmartDashboard.putNumber("Distance to Target Using Horizontal FOV", camera.getDistanceToTargetUsingHorizontalFov());
		
		SmartDashboard.putBoolean("Elevator Limit Switch", elevatorControl.getLimitSwitchState());
		SmartDashboard.putNumber("Elevator Position", elevatorControl.getPosition());
		SmartDashboard.putNumber("Elevator Enc Position", elevatorControl.getEncoderPosition());
		SmartDashboard.putBoolean("Elevator IsHoming?", elevatorControl.isHoming());
		SmartDashboard.putBoolean("Elevator IsMoving?", elevatorControl.isMoving());
		SmartDashboard.putNumber("Elevator Target", elevatorControl.getTarget());
		SmartDashboard.putBoolean("Elevator Has Been Homed?", elevatorControl.hasBeenHomed());
		SmartDashboard.putBoolean("Elevator isDown", elevatorControl.isDown());
		SmartDashboard.putBoolean("Elevator isMidway", elevatorControl.isMidway());
		SmartDashboard.putBoolean("Elevator isUp", elevatorControl.isUp());
		
		SmartDashboard.putBoolean("Hinge Limit Switch", hingeControl.getLimitSwitchState());
		SmartDashboard.putNumber("Hinge Position", hingeControl.getPosition());
		SmartDashboard.putNumber("Hinge Enc Position", hingeControl.getEncoderPosition());
		SmartDashboard.putBoolean("Hinge IsHoming?", hingeControl.isHoming());
		SmartDashboard.putBoolean("Hinge IsMoving?", hingeControl.isMoving());
		SmartDashboard.putNumber("Hinge Target", hingeControl.getTarget());
		SmartDashboard.putBoolean("Hinge Has Been Homed?", hingeControl.hasBeenHomed());
		SmartDashboard.putBoolean("Hinge isDown", hingeControl.isDown());
		SmartDashboard.putBoolean("Hinge isMidway", hingeControl.isMidway());
		SmartDashboard.putBoolean("Hinge isUp", hingeControl.isUp());
		
		SmartDashboard.putBoolean("Gyro Manually Calibrated?",hasGyroBeenManuallyCalibratedAtLeastOnce);
		//SmartDashboard.putNumber("PID Error", drivetrain.turnPidController.getError());
		//SmartDashboard.putNumber("PID Motor Value", drivetrain.turnPidController.get());
		//SmartDashboard.putBoolean("PID On Target", drivetrain.turnPidController.onTarget());
		
		SmartDashboard.putNumber("Tilt", accelerometer.getTilt());
		
		SmartDashboard.putString("First Switch", gameData.getAssignedPlateAtFirstSwitch().toString());
		SmartDashboard.putString("Scale", gameData.getAssignedPlateAtScale().toString());
		SmartDashboard.putString("Second Switch", gameData.getAssignedPlateAtSecondSwitch().toString());
		
		SmartDashboard.putNumber("Range to target", sonar.getRangeInInches());
		SmartDashboard.putNumber("Sonar Voltage", sonar.getVoltage()); 
		
		SmartDashboard.putBoolean("Grasper IsGrasping?", grasper.isGrasping());
		SmartDashboard.putBoolean("Grasper IsReleasing?", grasper.isReleasing());
		
		//SmartDashboard.putBoolean("Winch IsWinchingUp?", winchControl.isWinchingUp());
		
		SmartDashboard.putString("Auton selected", autonChooser.getSelected());	
		SmartDashboard.putString("Start position", startPositionChooser.getSelected());
		SmartDashboard.putString("Camera option", cameraOptionChooser.getSelected());
		SmartDashboard.putString("Sonar option", sonarOptionChooser.getSelected());
		SmartDashboard.putString("Release chosen", releaseChooser.getSelected());
	}
}
