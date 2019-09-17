/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem implements RobotMap{

  public WPI_TalonSRX leftMaster,leftSlave, rightMaster, rightSlave;
  public Encoder leftEncoder, rightEncoder;
  public Solenoid shifter;
  public boolean highGear = false;
  public double initAngle =0;
  public AHRS navx;


  public Drivetrain()
  {
    leftMaster = new WPI_TalonSRX(DRIVEMAP.LEFT_MASTER_ID);
    leftSlave = new WPI_TalonSRX(DRIVEMAP.LEFT_SLAVE_ID);
    rightMaster = new WPI_TalonSRX(DRIVEMAP.RIGHT_MASTER_ID);
    rightSlave = new WPI_TalonSRX(DRIVEMAP.RIGHT_SLAVE_ID);

    ConfigureTalon(leftMaster, DRIVEMAP.LEFT_MASTER_ID, true, false, leftMaster);
    ConfigureTalon(leftSlave, DRIVEMAP.LEFT_SLAVE_ID, true, true, leftMaster);

    ConfigureTalon(rightMaster, DRIVEMAP.RIGHT_MASTER_ID, true, false, rightMaster);
    ConfigureTalon(rightSlave, DRIVEMAP.RIGHT_SLAVE_ID, true, true, rightMaster);

    configPID(leftMaster, 1, 1, 1, 0);
    configPID(rightMaster, 1, 1, 1, 0);

    leftEncoder = new Encoder(SENSORMAP.LEFT_ENCODER_A, SENSORMAP.LEFT_ENCODER_B);
    rightEncoder = new Encoder(SENSORMAP.RIGHT_ENCODER_A, SENSORMAP.RIGHT_ENCODER_B);

    leftEncoder.setReverseDirection(true);

    shifter = new Solenoid(DRIVEMAP.SHIFTER_ID);

    navx = new AHRS(SerialPort.Port.kUSB1);
    resetnavx();
    initAngle = getFusedAngle();
    
  }

  public void turnDriveTrain(boolean turnLeft, double power)
  {
    double direction = turnLeft ? -1.0 : 1.0;
      arcadeDrive(ControlMode.PercentOutput,0, direction*power);
  }

  public void turnDriveTrain(double angle)
  {
    resetnavx();
    double power = (angle-navx.getAngle())/360;
    double direction = (navx.getAngle()>angle) ? -1 :1;
    while((angle+-10) != navx.getAngle())
    {arcadeDrive(ControlMode.PercentOutput,0, direction*power);}
  }

  public void navigateDrive(double y, double angle)
  {
    resetnavx();
    double turnPower =(angle-navx.getAngle())/360;
    while((angle+-10) != navx.getAngle())
  {
    arcadeDrive(ControlMode.PercentOutput, y, turnPower);
  }
    arcadeDrive(ControlMode.PercentOutput, y, 0);
  }

  public void tankDrive(ControlMode mode, double left, double right)
  {
    if(mode == ControlMode.Velocity)
    {left = left*DRIVEMAP.MAX_VEL; right = right*DRIVEMAP.MAX_VEL;}
    drive(mode, left, right);
  }

  public void tankDrive(double left, double right)
  {drive(ControlMode.PercentOutput, left, right);}
  
  public void arcadeDrive(ControlMode mode, double y, double z)
  {
    if(mode == ControlMode.Velocity)
    {y = y*DRIVEMAP.MAX_VEL; z = z*DRIVEMAP.MAX_VEL;} 
    double left = y+z, right = y-z;
    drive(mode, left, right);
  }

  public void arcadeDrive(double y, double z)
  {double left = y+z, right = y-z;  drive(ControlMode.PercentOutput, left, right);}
  
  public void drive(ControlMode mode, double left, double right)
  {
    leftMaster.set(mode, left); rightMaster.set(mode, right);
  }
  
  public void shiftGears()
  {
    shifter.set(!highGear);
  }
  public void autoShfiting()
  {
    while((leftEncoder.getRate() + rightEncoder.getRate()) < 4.5)
    {
      highGear = false;
    }
    while(leftEncoder.getRate() + rightEncoder.getRate() > 5)
    {
      highGear= true;
    }
    shifter.set(highGear);
  }

  public void stopMotors(){drive(ControlMode.PercentOutput, 0, 0);}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  

  public void ConfigureTalon(WPI_TalonSRX talon, int talon_id, boolean isLeft, boolean isSlave, WPI_TalonSRX masterSrx)
	{		
    talon.setInverted(isLeft);
		talon.setNeutralMode(NeutralMode.Brake);
		talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 10, 10);
		talon.setSelectedSensorPosition(0, 0, 10);
    
    talon.configPeakOutputForward(1, 10);
		talon.configPeakOutputReverse(-1, 10);
		talon.configNeutralDeadband(0.05, 10);
		talon.configNominalOutputForward(0, 10);
    talon.configNominalOutputReverse(0, 10);
    talon.enableCurrentLimit(false);
		talon.configContinuousCurrentLimit(0, 10);
		talon.configPeakCurrentDuration(0, 10);
		talon.configPeakCurrentLimit(0, 10);
		talon.configVoltageCompSaturation(12, 10);
		talon.enableVoltageCompensation(true);
		talon.configOpenloopRamp(0, 10);

		talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 10);
    talon.configVelocityMeasurementWindow(64, 10);
    
    if(isSlave){talon.follow(masterSrx);}
  }
  public void configPID(WPI_TalonSRX talon, int Kp, int Ki, int Kd, int Kff)
  {
    talon.config_kP(0, Kp, 10);
    talon.config_kI(0, Ki, 10);
    talon.config_kD(0, Kd, 10);
    talon.config_kF(0, Kff, 10);
    talon.configNominalOutputForward(.12, 10);
    talon.configNominalOutputReverse(.12, 10);
  }

  public void resetnavx()
	{navx.reset();}
	public double getFusedAngle()
	{return ((navx.getFusedHeading() + initAngle) % 360.0);}


  public double getnavxVelocityX()
  {return navx.getVelocityX();}

  public double getnavxVelocityY()
  {return navx.getVelocityY();}

  public double getnavxVelocityZ()
  {return navx.getVelocityZ();}
  
  public void resetDriveEncoders()
  {leftEncoder.reset();rightEncoder.reset();}

  public double getLeftDistance()
  {return leftEncoder.getDistance()/64;}

  public double getRightDistance()
  {return rightEncoder.getDistance()/64;}
  
  private static double rotationsToInches(double rotations)
  {return rotations * (4 * Math.PI);}

  private static double rpmToInchesPerSecond(double rpm) 
  {return rotationsToInches(rpm) / 60;}

  private static double inchesToRotations(double inches) 
  {return inches / (4 * Math.PI);}

  private static double inchesPerSecondToRpm(double inches_per_second) 
  {return inchesToRotations(inches_per_second) * 60;}

public void periodic()
{
  SmartDashboard.putNumber("Left Motor Percent",leftMaster.getMotorOutputPercent());
  SmartDashboard.putNumber("Right Motor Percent", rightMaster.getMotorOutputPercent());

  SmartDashboard.putNumber("Left Distance Traveled", getLeftDistance());
  SmartDashboard.putNumber("Right Distance Traveled", getRightDistance());

  SmartDashboard.putNumber("Robot Angle", getFusedAngle());
}  

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  
}
