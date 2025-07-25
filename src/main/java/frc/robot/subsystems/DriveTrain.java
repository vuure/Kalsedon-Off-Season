package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Vect;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class DriveTrain extends SubsystemBase {

  WPI_VictorSPX backright;
  WPI_VictorSPX frontright;
  WPI_TalonSRX frontleft;
  WPI_VictorSPX backleft;

  Encoder backrightEncoder;
  Encoder frontrightEncoder; 
  Encoder frontleftEncoder; 
  Encoder backleftEncoder;

  AnalogGyro gyro;

  MecanumDrive mDrive;

  MecanumDriveOdometry odometry;
  Pose2d pose;

  double radian;
  double degree;

  double magnitude;

  public DriveTrain(){

    backright = new WPI_VictorSPX(Constants.BackRightCAN_Num);
    frontright = new WPI_VictorSPX(Constants.FrontRightCAN_Num);
    frontleft = new WPI_TalonSRX(Constants.FrontLeftCAN_Num);
    backleft = new WPI_VictorSPX(Constants.BackLeftCAN_Num);

    backrightEncoder = new Encoder(Constants.backrightEncoder_A, Constants.backrightEncoder_B);
    backleftEncoder = new Encoder(Constants.backleftEncoder_A, Constants.backleftEncoder_B);
    frontrightEncoder = new Encoder(Constants.frontrightEncoder_A, Constants.frontrightEncoder_B);
    frontleftEncoder = new Encoder(Constants.frontleftEncoder_A, Constants.frontleftEncoder_B);

    gyro = new AnalogGyro(Constants.GyroPWM_Num);

    backright.setInverted(true);
    frontright.setInverted(true);

    mDrive = new MecanumDrive(frontleft, backleft, frontright, backright);

    Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d backRightLocation = new Translation2d(-0.381, -0.381);
    
    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    odometry = new MecanumDriveOdometry(
      kinematics,
      gyro.getRotation2d(),
      new MecanumDriveWheelPositions(
        frontleftEncoder.getDistance(), frontrightEncoder.getDistance(),
        backleftEncoder.getDistance(), backrightEncoder.getDistance()
      ),
      new Pose2d(5.0, 13.5, new Rotation2d())
    );
  }

  @Override
  public void periodic() {
    var wheelPositions = new MecanumDriveWheelPositions(
    frontleftEncoder.getDistance(), frontrightEncoder.getDistance(),
    backleftEncoder.getDistance(), backrightEncoder.getDistance());
    
    var gyroAngle = gyro.getRotation2d();

    pose = odometry.update(gyroAngle, wheelPositions);
  }

  public void driveWithJoystick(Joystick controller){
    mDrive.driveCartesian(-controller.getRawAxis(Constants.LeftY_Axis), -controller.getRawAxis(Constants.LeftX_Axis), controller.getRawAxis(Constants.RightX_Axis)/*, Rotation2d.fromDegrees(gyro.getAngle())*/);
  }

  public void setPower(double xspeed, double yspeed, double zspeed){
    mDrive.driveCartesian(xspeed, yspeed, zspeed);
  }

  public void polarDrive(double power, double angle, double rotation){
    mDrive.drivePolar(power, Rotation2d.fromDegrees(angle), rotation);

  }

  public Pose2d getPose(){
    return pose;
  }

  public void stop() {
    mDrive.stopMotor();
  }

  public Vect unitVectorBetweenTwoPoint(Translation2d point1, Translation2d point2){
  magnitude = point1.getDistance(point2);
  return new Vect((point1.getX() - point2.getX()) / magnitude, (point1.getY() - point2.getY()) / magnitude);
  }

  public double returnMagnitude() {
    return magnitude;
  }

  public double degreeBetweenTwoUnitVector(Vect vector1, Vect vector2){

    radian = Math.atan2(vector1.getY() - vector2.getY(), vector1.getX() - vector2.getX());

    degree = radian * 180 / Math.PI;

    return degree;
  }
}