package frc.robot.subsystems.simImplementations;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.DrivetrainInterface;

public class DrivetrainSim extends DrivetrainInterface {
  SimCANSparkMax left1 = new SimCANSparkMax(1, MotorType.kBrushless);
  SimCANSparkMax left2 = new SimCANSparkMax(2, MotorType.kBrushless);
  SimCANSparkMax right1 = new SimCANSparkMax(3, MotorType.kBrushless);
  SimCANSparkMax right2 = new SimCANSparkMax(4, MotorType.kBrushless);

  AnalogGyro gyro = new AnalogGyro(0);
  AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

  Field2d field = new Field2d();

  DifferentialDrivetrainSim drivetrainSim =
      new DifferentialDrivetrainSim(
          Constants.DrivetrainConstants.motorsPerSide,
          Constants.DrivetrainConstants.gearRatio,
          Constants.DrivetrainConstants.estimatedMomentOfInertia,
          Constants.DrivetrainConstants.estimatedMass,
          Constants.DrivetrainConstants.wheelRadius,
          Constants.DrivetrainConstants.trackWidth,
          VecBuilder.fill(
              Constants.SimConstants.stdDevX,
              Constants.SimConstants.stdDevY,
              Constants.SimConstants.stdDevHeading,
              Constants.SimConstants.stdDevLeftVelocity,
              Constants.SimConstants.stdDevRightVelocity,
              Constants.SimConstants.stdDevLeftPosition,
              Constants.SimConstants.stdDevRightPosition));

  DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  public DrivetrainSim() {
    left1.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.distancePerPulse);
    right1.getEncoder().setPositionConversionFactor(Constants.DrivetrainConstants.distancePerPulse);
    left1
        .getEncoder()
        .setVelocityConversionFactor(Constants.DrivetrainConstants.distancePerPulse / 60);
    right1
        .getEncoder()
        .setVelocityConversionFactor(Constants.DrivetrainConstants.distancePerPulse / 60);

    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        getLeftEncoder().getPosition(),
        getRightEncoder().getPosition());

    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putData(
        "Odometry",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty(
                "X", () -> odometry.getPoseMeters().getTranslation().getX(), (z) -> {});
            builder.addDoubleProperty(
                "Y", () -> odometry.getPoseMeters().getTranslation().getY(), (z) -> {});
            builder.addDoubleProperty(
                "H", () -> odometry.getPoseMeters().getRotation().getDegrees(), (z) -> {});
          }
        });
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(
        left1.getAppliedOutput() * RobotController.getInputVoltage(),
        right1.getAppliedOutput() * RobotController.getInputVoltage());
    drivetrainSim.update(0.02);

    left1.getEncoder().setPosition(drivetrainSim.getLeftPositionMeters());
    left1.setVelocity(drivetrainSim.getLeftVelocityMetersPerSecond());
    right1.getEncoder().setPosition(drivetrainSim.getRightPositionMeters());
    right1.setVelocity(drivetrainSim.getRightVelocityMetersPerSecond());

    gyroSim.setAngle(drivetrainSim.getHeading().getDegrees());
  }

  @Override
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public double getHeading() {
    return gyro.getAngle();
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftEncoder().getVelocity(), getRightEncoder().getVelocity());
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    getLeftEncoder().setPosition(0);
    getRightEncoder().setPosition(0);
  }

  @Override
  public void applyLeftRightVoltage(double leftVolts, double rightVolts) {
    this.applyLeftRightDutyCycle(
        leftVolts / RobotController.getBatteryVoltage(),
        rightVolts / RobotController.getBatteryVoltage());
  }

  @Override
  public void applyLeftRightDutyCycle(double leftDutyCycle, double rightDutyCycle) {
    left1.getSimDeviceSim().getDouble("Applied Output").set(leftDutyCycle);
    right1.getSimDeviceSim().getDouble("Applied Output").set(rightDutyCycle);
    // toApplyLeft = leftDutyCycle;
    // toApplyRight = rightDutyCycle;
  }

  public void arcadeDrive(double translation, double rotation) {
    DifferentialDriveWheelSpeeds wheelSpeeds =
        Constants.DrivetrainConstants.kinematics.toWheelSpeeds(
            new ChassisSpeeds(translation * Constants.DrivetrainConstants.maxSpeed, 0, rotation));

    this.applyLeftRightDutyCycle(
        wheelSpeeds.leftMetersPerSecond / Constants.DrivetrainConstants.maxSpeed,
        wheelSpeeds.rightMetersPerSecond / Constants.DrivetrainConstants.maxSpeed);
  }

  @Override
  public RelativeEncoder getLeftEncoder() {
    return left1.getEncoder();
  }

  @Override
  public RelativeEncoder getRightEncoder() {
    return right1.getEncoder();
  }
}
