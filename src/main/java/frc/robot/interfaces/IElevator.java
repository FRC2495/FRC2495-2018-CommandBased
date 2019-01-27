package frc.robot.interfaces;

import edu.wpi.first.wpilibj.Joystick;

public interface IElevator {

	// returns the state of the limit switch
	public boolean getLimitSwitchState();
	
	// homes the elevator
	// This is done in two steps:
	// step 1: if not already at the switch, we go down slowly until we hit the limit switch.
	// step 2: we go back up a little and mark the position as the virtual/logical zero.
	public void home();
	
	// this method need to be called to assess the homing progress
	// (and it takes care of going to step 2 if needed)
	public boolean checkHome();
	
	// do not use in teleop - for auton only
	public void waitHome();
	
	// This method should be called to assess the progress of a move
	public boolean tripleCheckMove();

	// do not use in teleop - for auton only
	public void waitMove();
	
	public void moveUp();
	
	public void moveMidway();
	
	public void moveDown();

	public double getPosition();

	public double getEncoderPosition();

	public boolean isHoming();
	
	public boolean isMoving();
	
	public boolean isUp();
	
	public boolean isDown();
	
	public boolean isMidway();

	public void stay();
	
	public void stop();
		
	// for debug purpose only
	public void joystickControl(Joystick joystick);
	
	public double getTarget();
	
	public boolean hasBeenHomed();

}
