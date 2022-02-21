// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.interfaces.DrivetrainInterface;
import frc.robot.subsystems.simImplementations.DrivetrainSim;
import frc.robot.util.Ramsete;
import java.util.List;

public class RobotContainer {

  DrivetrainInterface drivetrain = new DrivetrainSim();
  Joystick joystick = new Joystick(0);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              drivetrain.arcadeDrive(joystick.getRawAxis(1), joystick.getRawAxis(0));
            },
            drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return Ramsete.makeRamseteCommand(
        Ramsete.makeTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(45))),
            0,
            0,
            5,
            false),
        drivetrain);
  }
}
