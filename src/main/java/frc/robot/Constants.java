package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/*
  UPDATE WPILib
  UPDATE Driver Station
  UPDATE Libraries
  AND Have Fun!
*/

public final class Constants {
  //DIO number for Encoders
  public static final int backleftEncoder_A = 0;
  public static final int backleftEncoder_B = 1;

  public static final int backrightEncoder_A = 2;
  public static final int backrightEncoder_B = 3;

  public static final int frontleftEncoder_A = 4;
  public static final int frontleftEncoder_B = 5;

  public static final int frontrightEncoder_A = 6;
  public static final int frontrightEncoder_B = 7;

  //PWM Number for Intake 
  public static final int Intake1PWM_Num = 0;
  public static final int Intake2PWM_Num = 1;

  //CAN Number for Elevator
  public static final int ElevatorCAN_Num = 1;

  //CAN ID's for Mecanum Drive Motors
  public static final int BackRightCAN_Num = 23;
  public static final int BackLeftCAN_Num = 25;
  public static final int FrontLeftCAN_Num = 24;
  public static final int FrontRightCAN_Num = 22;

  //Driver Station Axis IDs for Controller1 (A.K.A the DriveTrain Controller)
  public static final int LeftY_Axis = 1;
  public static final int LeftX_Axis = 0;
  public static final int RightX_Axis = 4;

  //Axis IDs for Controller1 for Arm
  public static final int LeftTrigger_Axis = 2;
  public static final int RightTrigger_Axis = 3;

  //Button IDs for Coral Intake
  public static final int CoralIntakeGetButton_Num = 1;
  public static final int CoralIntakeLeaveButton_Num = 2;

  //Gyro Port
  public static final int GyroPWM_Num = 0;

  //Drive Joystick Port
  public static final int DriveJoystickPort_Num = 0;

  //Arm PID Values
  public static final double arm_P = 0;
  public static final double arm_I = 0;
  public static final double arm_D = 0;

  //Arm Encoder Positions
  public static final double L1_Position = 0;
  public static final double L2_Position = 0;
  public static final double L3_Position = 0;
  public static final double L4_Position = 0;

  public static final double Bottom_Position = 0;
  public static final double Top_Position = 0;

  //Test
  public static final double DriveForwardTime = 3;
  public static final double AutoSpeed = 1;

  public static final PIDController mecanumDrivePIDController = new PIDController(5.0, 0.0, 0.0);
  public static final PIDController mecanumRotatePIDController = new PIDController(5.0, 0.0, 0.0);
}