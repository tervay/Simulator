package frc.robot.subsystems.interfaces;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DrivetrainInterface extends SubsystemBase {
  public abstract Pose2d getPose();

  public abstract double getHeading();

  public abstract DifferentialDriveWheelSpeeds getWheelSpeeds();

  public abstract void resetOdometry();

  public abstract void applyLeftRightVoltage(double leftVolts, double rightVolts);

  public abstract void applyLeftRightDutyCycle(double leftDutyCycle, double rightDutyCycle);

  public abstract void arcadeDrive(double translation, double rotation);

  public abstract RelativeEncoder getLeftEncoder();

  public abstract RelativeEncoder getRightEncoder();
}
