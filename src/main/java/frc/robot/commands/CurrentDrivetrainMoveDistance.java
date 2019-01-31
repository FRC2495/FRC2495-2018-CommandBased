// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;

/**
 *
 */
public class CurrentDrivetrainMoveDistance extends ConditionalCommand {

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
	public CurrentDrivetrainMoveDistance(double distance) {
	  super(new DrivetrainMoveDistance(distance), new MiniDrivetrainMoveDistance(distance));
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
	}

	@Override
	protected boolean condition(){
		return Robot.largeDriveTrainSelected;
	}
}
