/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.CONTROLLERMAP;

public class DriveJ extends Command implements CONTROLLERMAP{
  public XboxController controller = new XboxController(CONTROLLER_ID);
  public int isTank;


  public DriveJ() {
   requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double powerPercent = getButtonValue(BUMPER_LEFT)||getButtonValue(BUMPER_RIGHT) ? .75 : 1;
    double triggerPercent = (1-(getAxisValue(RIGHT_TRIGGER)));
    double powerChange = powerPercent*triggerPercent;

    double leftStickY = getAxisValue(LEFT_Y_AXIS)* powerChange;
    double leftStickX = getAxisValue(LEFT_X_AXIS) *powerChange;
    double rightStickY = getAxisValue(RIGHT_Y_AXIS) *powerChange;

    if (getButtonValue(STICK_LEFT) && isTank == 0)
    { isTank++;}
    else {isTank--;}
    if(isTank >0)
    {
      Robot.drivetrain.tankDrive(ControlMode.Velocity,leftStickY, rightStickY);
    }
    else
    {
      Robot.drivetrain.arcadeDrive(ControlMode.Velocity,leftStickX, rightStickY);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  public double getAxisValue(int Axis)
  {
    return Math.abs(controller.getRawAxis(Axis)) <.15 ? 0 :controller.getRawAxis(Axis);
  }
  public boolean getButtonValue(int button)
  {
    return controller.getRawButton(button);
  }
}
