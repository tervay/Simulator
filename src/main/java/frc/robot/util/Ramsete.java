package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class Ramsete {
  public static Command makeRamseteCommand(Trajectory trajectory, DrivetrainInterface drivetrain) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              drivetrain.resetOdometry(trajectory.getInitialPose());
            }),
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

  public static List<Trajectory> makeTrajectories(
      Pose2d startingPoint,
      List<PoseTrigger> poseTriggers,
      Pose2d endingPoint,
      double maxVelocity,
      boolean reversed) {
    ArrayList<Pose2d> poses =
        new ArrayList<>() {
          {
            add(startingPoint);
            addAll(poseTriggers.stream().map(pt -> pt.pose).collect(Collectors.toList()));
            add(endingPoint);
          }
        };

    System.out.println(stringifyPoses(poses));

    Trajectory fullTrajectory = makeTrajectory(poses, 0, 0, maxVelocity, reversed);

    ArrayList<State> closestStates =
        new ArrayList<>() {
          {
            add(fullTrajectory.getStates().get(0));
          }
        };
    poseTriggers.forEach(
        pt -> {
          List<State> states = fullTrajectory.getStates();
          closestStates.add(
              states.stream()
                  .min(
                      Comparator.comparingDouble(
                          s -> pt.pose.getTranslation().getDistance(s.poseMeters.getTranslation())))
                  .get());
        });

    System.out.println("Splitting based on " + closestStates.size() + " states");

    closestStates.add(fullTrajectory.getStates().get(fullTrajectory.getStates().size() - 1));

    return splitTrajectory(closestStates, maxVelocity, reversed);
  }

  private static List<Trajectory> splitTrajectory(
      List<State> states, double maxVelocity, boolean reversed) {
    ArrayList<Trajectory> trajectories = new ArrayList<>();
    for (int i = 0; i < states.size() - 1; i++) {

      System.out.println(states.get(i).poseMeters);
      System.out.println(states.get(i + 1).poseMeters);
      System.out.println("----");

      trajectories.add(
          makeTrajectory(
              List.of(states.get(i).poseMeters, states.get(i + 1).poseMeters),
              states.get(i).velocityMetersPerSecond,
              states.get(i + 1).velocityMetersPerSecond,
              maxVelocity,
              reversed));
    }

    return trajectories;
  }

  public static Command makeDynamicCommand(
      Pose2d startingPoint,
      List<PoseTrigger> poseTriggers,
      Pose2d endingPoint,
      double maxVelocity,
      boolean reversed,
      DrivetrainInterface drivetrain) {
    SequentialCommandGroup commands = new SequentialCommandGroup();
    var trajectories =
        makeTrajectories(startingPoint, poseTriggers, endingPoint, maxVelocity, reversed);

    int i = 0;
    for (Trajectory t : trajectories) {
      ParallelCommandGroup parallelCommandGroup =
          new ParallelCommandGroup(makeRamseteCommand(t, drivetrain));

      for (State s : t.getStates()) {
        System.out.println(
            s.timeSeconds
                + ","
                + s.poseMeters.getX()
                + ","
                + s.poseMeters.getY()
                + ","
                + s.poseMeters.getRotation().getDegrees()
                + ","
                + s.velocityMetersPerSecond);
      }

      System.out.println("-----------");

      if (i < poseTriggers.size()) {
        parallelCommandGroup.addCommands(poseTriggers.get(i).command);
      }

      commands.addCommands(parallelCommandGroup);
      i++;
    }

    return commands;
  }

  public static String stringifyPoses(List<Pose2d> poses) {
    return poses.stream().map(e -> e.toString()).reduce(" / ", String::concat);
  }
}
