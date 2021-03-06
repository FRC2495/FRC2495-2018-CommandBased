package frc.robot.subsystems;

import java.util.Calendar;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.interfaces.*;
import frc.robot.Ports;


// a class to raise the outer/main drivetrain (by lowering the inner/mini drivetrain)
public class Jack extends Subsystem implements IJack{
	
	static final int WAIT_MS = 1000;
	
	DoubleSolenoid downup;

	/*public enum Position {
		LARGE_DRIVETRAIN, // outer drivetrain is down   //Previously UP
		MINI_DRIVETRAIN, // outer drivetrain is up     //Previously DOWN
		UNKNOWN;
    }*/

	public Jack() {
		// the double solenoid valve will send compressed air from the tank wherever needed
		downup = new DoubleSolenoid(Ports.CAN.PCM, Ports.PCM.JACK_DOWN, Ports.PCM.JACK_UP); // make sure ports are properly sets in Ports.java	
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

	public void setPosition(Position pos)
	{
		switch(pos)
		{
			case MINI_DRIVETRAIN:
			{
				downup.set(DoubleSolenoid.Value.kReverse); // adjust direction if needed
				break;
			}
			case LARGE_DRIVETRAIN:
			{
				downup.set(DoubleSolenoid.Value.kForward); // adjust direction if needed
				break;
			}
			default:
			{
				// do nothing
			}
		}
	}

	public Position getPosition()
	{
		DoubleSolenoid.Value value = downup.get();
		
		switch(value)
		{
			case kReverse:
			{
				return Position.MINI_DRIVETRAIN;
			}
			case kForward:
			{
				return Position.LARGE_DRIVETRAIN;
			}
			default:
			{
				return Position.UNKNOWN;
			}
		}
	}
	
	public void waitSetPosition() {
		long start = Calendar.getInstance().getTimeInMillis();

		while (true) { 		
			if (!DriverStation.getInstance().isAutonomous()
					|| Calendar.getInstance().getTimeInMillis() - start >= WAIT_MS) {
				System.out.println("Wait is over");
				break;
			}

			try {
				Thread.sleep(20); // sleeps a little
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			//robot.updateToSmartDash();
		}
	}
}
