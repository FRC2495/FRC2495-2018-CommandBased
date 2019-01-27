package frc.robot.interfaces;

import edu.wpi.first.wpilibj.Joystick;

public interface IMiniDrivetrain {
	
	// this method needs to be paired with checkMoveUsingCameraPidController()
	public void moveUsingCameraPidController();
	
	public boolean tripleCheckMoveUsingCameraPidController();
	
	// do not use in teleop - for auton only
	public void waitMoveUsingCameraPidController();
	
		
	// this method needs to be paired with checkMoveDistance()
	public void moveDistance(double dist);
	
	public boolean tripleCheckMoveDistance();
	
	// do not use in teleop - for auton only
	public void waitMoveDistance();
	
	
	public void stop();
	

	public void setPIDParameters();
	
	// NOTE THAT THIS METHOD WILL IMPACT BOTH OPEN AND CLOSED LOOP MODES
	public void setNominalAndPeakOutputs(double peakOutput);
	
	public void joystickControl(Joystick joyLeft, Joystick joyRight, boolean held);
	
	public int getRightEncoderPosition();

	public int getLeftEncoderPosition();

	public int getRightPosition();

	public int getLeftPosition();
	
	public boolean isMoving();
	
	public boolean isMovingUsingCamera();
			
	// MAKE SURE THAT YOU ARE NOT IN A CLOSED LOOP CONTROL MODE BEFORE CALLING THIS METHOD.
	// OTHERWISE THIS IS EQUIVALENT TO MOVING TO THE DISTANCE TO THE CURRENT ZERO IN REVERSE! 
	public void resetEncoders();
}


