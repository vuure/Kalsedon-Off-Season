package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Vect;
import frc.robot.subsystems.DriveTrain;

public class FollowPath extends Command {

  private final DriveTrain driveTrain;
  private final List<Pose2d> path;
  private final boolean escapeButton;
  boolean codeFinish;

  Pose2d robotpose;
  PIDController robotdrivePID = Constants.mecanumDrivePIDController;
  PIDController robotrotatePID = Constants.mecanumRotatePIDController;

  Vect robotHeadingUnitVector;
  Vect robotToPointUnitVector;
  double polarDriveDegree;

  double magnitude;

  Pose2d point;

  public FollowPath(DriveTrain dt, List<Pose2d> ph, boolean esc) {
    driveTrain = dt;
    path = ph;
    escapeButton = esc;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    codeFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(path.size() == 0)){
      if(!(0.05 >= magnitude && 0.5 >= Math.abs(point.getRotation().getDegrees() - robotpose.getRotation().getDegrees()))){
        point = path.get(0);
        robotpose = driveTrain.getPose();
        robotHeadingUnitVector = new Vect(robotpose.getRotation().getCos(), robotpose.getRotation().getSin());
        robotToPointUnitVector = driveTrain.unitVectorBetweenTwoPoint(robotpose.getTranslation(), point.getTranslation());

        magnitude = driveTrain.returnMagnitude();

        polarDriveDegree = driveTrain.degreeBetweenTwoUnitVector(robotHeadingUnitVector, robotToPointUnitVector);

        driveTrain.polarDrive(robotdrivePID.calculate(magnitude, 0),polarDriveDegree, robotdrivePID.calculate(robotpose.getRotation().getDegrees(), point.getRotation().getDegrees()));
      }else{
        path.remove(0);
      }
    } else {
      codeFinish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return codeFinish;
  }
}
