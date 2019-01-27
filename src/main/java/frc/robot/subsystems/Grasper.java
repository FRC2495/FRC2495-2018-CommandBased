/**
 * 
 */
package frc.robot.subsystems;

import java.util.Calendar;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.interfaces.*;
//import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.sensors.Sonar;


/**
 * @author Joshua
 *
 */
public class Grasper extends Subsystem implements IGrasper{

	/**
	 * 
	 */
	static final double MAX_PCT_OUTPUT = 1.0;
	static final double ALMOST_MAX_PCT_OUTPUT = 0.5;
	static final double HALF_PCT_OUTPUT = 0.5;
	static final double REDUCED_PCT_OUTPUT = 0.4;
	
	static final int WAIT_MS = 1000;
	static final int TIMEOUT_MS = 5000;

	static final int TALON_TIMEOUT_MS = 10;

	static final int GRASP_DISTANCE_INCHES = 13;
	static final int RELEASE_DISTANCE_INCHES = 17;
	
	BaseMotorController grasperLeft , grasperRight; 
	Sonar sonar;
	
	// shared grasp and release settings
	private int onTargetCount; // counter indicating how many times/iterations we were on target
	private final static int ON_TARGET_MINIMUM_COUNT = 25; // number of times/iterations we need to be on target to really be on target
	
	boolean isGrasping;
	boolean isReleasing;
	
	Robot robot;
	
	
	public Grasper(BaseMotorController grasperLeft_in, BaseMotorController grasperRight_in, Robot robot_in) {
		
		grasperLeft = grasperLeft_in;
		grasperRight = grasperRight_in;
		
		robot = robot_in;
		
		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.
		grasperLeft.setNeutralMode(NeutralMode.Brake);
		grasperRight.setNeutralMode(NeutralMode.Brake);
		
		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked).
		//this might me wrong =j
		grasperLeft.setInverted(false);
		grasperRight.setInverted(true);
		
		// Both the Talon SRX and Victor SPX have a follower feature that allows the motor controllers to mimic another motor controller's output.
		// Users will still need to set the motor controller's direction, and neutral mode.
		// The method follow() allows users to create a motor controller follower of not only the same model, but also other models
		// , talon to talon, victor to victor, talon to victor, and victor to talon.
		grasperRight.follow(grasperLeft);
		
		// set peak output to max in case if had been reduced previously
		setNominalAndPeakOutputs(MAX_PCT_OUTPUT);
	}
	
	public Grasper(BaseMotorController grasperLeft_in, BaseMotorController grasperRight_in, Sonar sonar_in, Robot robot_in) {
		this(grasperLeft_in, grasperRight_in, robot_in);
		
		sonar = sonar_in;
	}
	
    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

	public void grasp() {
		grasperLeft.set(ControlMode.PercentOutput, REDUCED_PCT_OUTPUT);
		
		isGrasping = true;
		isReleasing = false;
		onTargetCount = 0;
	}
	
	public void release() {
		grasperLeft.set(ControlMode.PercentOutput, -ALMOST_MAX_PCT_OUTPUT);
		
		isReleasing = true;
		isGrasping = false;
		onTargetCount = 0;
	}
	
	public void stop() {
		grasperLeft.set(ControlMode.PercentOutput, 0);
		
		isGrasping = false;
		isReleasing = false;
	}
	
	// do not use in teleop - for auton only
	// This version does NOT rely on the sonar. Use only if sonar does not fulfill expectations.
	public void waitGraspOrRelease() {
		long start = Calendar.getInstance().getTimeInMillis();

		while (true) { 		
			if (!DriverStation.getInstance().isAutonomous()
					|| Calendar.getInstance().getTimeInMillis() - start >= WAIT_MS) {
				System.out.println("Wait is over");
				stop();
				break;
			}

			try {
				Thread.sleep(20); // sleeps a little
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			robot.updateToSmartDash();
		}
	}
	
	public boolean tripleCheckGraspUsingSonar() {
		if (sonar != null && isGrasping) {
						
			boolean isOnTarget = sonar.getRangeInInches() < GRASP_DISTANCE_INCHES;
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCount++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCount > 0) { // even though we were on target at least once during a previous iteration
					onTargetCount = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (grasping).");
				} else {
					// we are definitely moving
					//System.out.println("Grasping. Sonar range: " + sonar.getRangeInInches());
				}
			}
			
	        if (onTargetCount > ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
	        	isGrasping = false;
	        }
			
			if (!isGrasping) {
				System.out.println("You have reached the target (grasping).");
				stop();				 
			}
		}
		return isGrasping;
	}
	
	// do not use in teleop - for auton only
	public void waitGraspUsingSonar() {
		long start = Calendar.getInstance().getTimeInMillis();
		
		while (tripleCheckGraspUsingSonar()) {
			if (!DriverStation.getInstance().isAutonomous()
					|| Calendar.getInstance().getTimeInMillis() - start >= TIMEOUT_MS) {
				System.out.println("You went over the time limit (grasping)");
				stop();
				break;
			}
			
			try {
				Thread.sleep(20); // sleeps a little
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			robot.updateToSmartDash();
		}
	}
	
	public boolean tripleCheckReleaseUsingSonar() {
		if (sonar != null && isReleasing) {
						
			boolean isOnTarget = sonar.getRangeInInches() > RELEASE_DISTANCE_INCHES;
			
			if (isOnTarget) { // if we are on target in this iteration 
				onTargetCount++; // we increase the counter
			} else { // if we are not on target in this iteration
				if (onTargetCount > 0) { // even though we were on target at least once during a previous iteration
					onTargetCount = 0; // we reset the counter as we are not on target anymore
					System.out.println("Triple-check failed (releasing).");
				} else {
					// we are definitely moving
					//System.out.println("Releasing. Sonar range: " + sonar.getRangeInInches());
				}
			}
			
	        if (onTargetCount > ON_TARGET_MINIMUM_COUNT) { // if we have met the minimum
	        	isReleasing = false;
	        }
			
			if (!isReleasing) {
				System.out.println("You have reached the target (releasing).");
				stop();				 
			}
		}
		return isReleasing;
	}
	
	// do not use in teleop - for auton only
	public void waitReleaseUsingSonar() {
		long start = Calendar.getInstance().getTimeInMillis();
		
		while (tripleCheckReleaseUsingSonar()) {
			if (!DriverStation.getInstance().isAutonomous()
					|| Calendar.getInstance().getTimeInMillis() - start >= TIMEOUT_MS) {
				System.out.println("You went over the time limit (releasing)");
				stop();
				break;
			}
			
			try {
				Thread.sleep(20); // sleeps a little
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			robot.updateToSmartDash();
		}
	}
		
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setNominalAndPeakOutputs(double peakOutput)
	{
		grasperLeft.configPeakOutputForward(peakOutput, TALON_TIMEOUT_MS);
		grasperLeft.configPeakOutputReverse(-peakOutput, TALON_TIMEOUT_MS);
		grasperRight.configPeakOutputForward(peakOutput, TALON_TIMEOUT_MS);
		grasperRight.configPeakOutputReverse(-peakOutput, TALON_TIMEOUT_MS);
		
		grasperRight.configNominalOutputForward(0, TALON_TIMEOUT_MS);
		grasperLeft.configNominalOutputForward(0, TALON_TIMEOUT_MS);
		grasperRight.configNominalOutputReverse(0, TALON_TIMEOUT_MS);
		grasperLeft.configNominalOutputReverse(0, TALON_TIMEOUT_MS);
	}
	
	public boolean isGrasping() {
		return isGrasping;
	}
	
	public boolean isReleasing(){
		return isReleasing;
	}

	// for debug purpose only
	public void joystickControl(Joystick joystick)
	{
		grasperLeft.set(ControlMode.PercentOutput, joystick.getY());
	}
}










