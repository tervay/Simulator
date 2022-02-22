package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PoseTrigger {
  public Pose2d pose;
  public double withinDistance;
  public Command command;

  public PoseTrigger(Pose2d pose) {
    this.pose = pose;
    withinDistance = Units.inchesToMeters(1);
    command = new InstantCommand();
  }

  public PoseTrigger within(double distance) {
    withinDistance = distance;
    return this;
  }

  public PoseTrigger run(Command command) {
    this.command = command;
    return this;
  }
}
