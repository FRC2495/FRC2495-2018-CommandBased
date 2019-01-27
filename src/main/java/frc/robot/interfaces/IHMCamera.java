package frc.robot.interfaces;

import edu.wpi.first.wpilibj.PIDSourceType;

public interface IHMCamera {

	public boolean isCoherent();

	public int getNumberOfTargets();

	public boolean acquireTargets(boolean waitForNewInfo);
	
	public boolean checkForCube();

	public double getDistanceToTargetUsingVerticalFov();
	
	public double getDistanceToTargetUsingHorizontalFov();

	public double getAngleToTurnToTarget();
	
	public double getPixelDisplacementToCenterToTarget();

	public double[] getArea();

	public double[] getWidth();

	public double[] getHeight();

	public double[] getCenterX();

	public double[] getCenterY();
	
	public void setPIDSourceType(PIDSourceType pidSource);
	
	public PIDSourceType getPIDSourceType();
	
	public double pidGet();
}
