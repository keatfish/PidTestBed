/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public interface RobotMap {

  public interface DRIVEMAP
  {
    public static final int 
        LEFT_MASTER_ID = 11,
        LEFT_SLAVE_ID = 13,
        RIGHT_MASTER_ID = 12,
        RIGHT_SLAVE_ID = 14;
    
    public static final int
        WHEEL_DIAMETER = 4,
        SHIFTER_ID = 0,
        MAX_VEL = 15;
    public static final double 
        TICKS2REVS = 64;

  }
  public interface CONTROLLERMAP
  {
    public static final int
    CONTROLLER_ID =3,

    LEFT_X_AXIS = 0,
    LEFT_Y_AXIS = 1,
    LEFT_TRIGGER = 2,
    RIGHT_TRIGGER = 3,
    RIGHT_X_AXIS = 4,
    RIGHT_Y_AXIS = 5;

    public static final int 
    BUTTON_A = 0,
    BUTTON_Y = 1,
    BUTTON_X = 2, 
    BUTTON_B = 3,
    BUTTON_BACK = 4,
    BUTTON_START = 5,
    BUMPER_LEFT = 6,
    BUMPER_RIGHT = 7,
    STICK_LEFT = 8,
    STICK_RIGHT = 9;

  }

  public interface SENSORMAP
  {
    public static final int 
        LEFT_ENCODER_A = 0,
        LEFT_ENCODER_B = 1,

        RIGHT_ENCODER_A =2,
        RIGHT_ENCODER_B = 3;
  }
  public interface CONSTANTS 
  {
    public static final String PROFILE_PATH = "/home/lvuser/";

  }
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
