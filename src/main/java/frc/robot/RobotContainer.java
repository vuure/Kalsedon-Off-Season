package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.MoveToPoint;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

  private final DriveTrain driveTrain = new DriveTrain();

  public static Joystick mainJoystick = new Joystick(Constants.DriveJoystickPort_Num);

  private final DriveWithJoystick driveWithJoysticks = new DriveWithJoystick(driveTrain, mainJoystick);

  public RobotContainer() {
    
    driveTrain.setDefaultCommand(driveWithJoysticks);
    
    driveWithJoysticks.addRequirements(driveTrain);

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new MoveToPoint(driveTrain, new Pose2d(1,1,Rotation2d.fromDegrees(180)), false),
      new MoveToPoint(driveTrain, new Pose2d(0,0,Rotation2d.fromDegrees(0)), false)
    );
  }
}
