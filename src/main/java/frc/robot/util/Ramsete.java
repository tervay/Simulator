package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import java.util.List;

public class Ramsete {
  public static Command makeRamseteCommand(Trajectory trajectory, DrivetrainInterface drivetrain) {
    return new SequentialCommandGroup(
        new RamseteCommand(
            trajectory,
            drivetrain::getPose,
            new RamseteController(2, 0.7),
            Constants.DrivetrainConstants.ramseteFF,
            Constants.DrivetrainConstants.kinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.DrivetrainConstants.kP, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kP, 0, 0),
            drivetrain::applyLeftRightVoltage,
            drivetrain));
  }

  public static Trajectory makeTrajectory(
      List<Pose2d> waypoints,
      double startVelocity,
      double endVelocity,
      double maxVelocity,
      boolean reversed) {
    return TrajectoryGenerator.generateTrajectory(
        waypoints,
        new TrajectoryConfig(maxVelocity, maxVelocity)
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .setReversed(reversed)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    Constants.DrivetrainConstants.ramseteFF,
                    Constants.DrivetrainConstants.kinematics,
                    10)));
  }
}
