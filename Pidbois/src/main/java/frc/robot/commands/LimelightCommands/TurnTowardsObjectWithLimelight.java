package frc.robot.commands.LimelightCommands;

import frc.robot.Robot;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnTowardsObjectWithLimelight extends Command {

	private double[] limelightData = new double[Limelight.NUMBER_OF_LIMELIGHT_CHARACTERISTICS];
	
	private boolean targetExists = false;
	private final double TURNING_MOTOR_POWER = .35;
	private final double X_THRESHOLD_TO_STOP_TURNING = 20.0;
	
    public TurnTowardsObjectWithLimelight()
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.limelight);
    	requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	//  Initializing Limelight
    	Robot.limelight.SetCameraPipeline(Limelight.YELLOW_BLOCK_PIPELINE);
    	Robot.limelight.SetCameraMode(Limelight.VISION_PROCESSOR);
    	Robot.limelight.SetLEDMode(Limelight.LED_OFF);  //  Turns off LED to Track the Yellow Block
    	
    	targetExists = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute()
    {
    	targetExists = Robot.limelight.isTargetsExist();
    	limelightData = Robot.limelight.GetLimelightData();
		
		if (targetExists)  // If Target is to the Right, turn to the Right
    	{
    		Robot.drivetrain.turnDriveTrain(limelightData[Limelight.HORIZONTAL_OFFSET]);
    	}
    	else {Robot.drivetrain.stopMotors();}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
        return (limelightData[Limelight.HORIZONTAL_OFFSET] <= X_THRESHOLD_TO_STOP_TURNING 
        		&& limelightData[Limelight.HORIZONTAL_OFFSET] >= -X_THRESHOLD_TO_STOP_TURNING
        		&& limelightData[Limelight.HORIZONTAL_OFFSET] != 0);
    }

    // Called once after isFinished returns true
    protected void end()
    {
    	Robot.drivetrain.stopMotors();
    	targetExists = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted()
    {
    	this.end();
    }
}
