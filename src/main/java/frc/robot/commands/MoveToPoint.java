package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vect;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Translation2d;

public class MoveToPoint extends Command {
  private final DriveTrain driveTrain;
  private final Pose2d point;
  private final boolean escapeButton;

  Pose2d robotpose;
  PIDController robotdrivePID = Constants.mecanumDrivePIDController;
  PIDController robotrotatePID = Constants.mecanumRotatePIDController;

  double radian;
  double degree;

  Vect robotHeadingUnitVector;
  Vect robotToPointUnitVector;

  double polarDriveDegree;

  double magnitude;

  public MoveToPoint(DriveTrain dt, Pose2d pt, boolean esc) {
    driveTrain = dt;
    point = pt;
    escapeButton = esc;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotpose = driveTrain.getPose();
    robotHeadingUnitVector = new Vect(robotpose.getRotation().getCos(), robotpose.getRotation().getSin());
    robotToPointUnitVector = driveTrain.unitVectorBetweenTwoPoint(robotpose.getTranslation(), point.getTranslation());

    polarDriveDegree = driveTrain.degreeBetweenTwoUnitVector(robotHeadingUnitVector, robotToPointUnitVector);

    magnitude = driveTrain.returnMagnitude();

    driveTrain.polarDrive(robotdrivePID.calculate(magnitude, 0),polarDriveDegree, robotdrivePID.calculate(robotpose.getRotation().getDegrees(), point.getRotation().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (0.05 >= magnitude && 0.5 >= Math.abs(point.getRotation().getDegrees() - robotpose.getRotation().getDegrees())) || escapeButton;
  }

}