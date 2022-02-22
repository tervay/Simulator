// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class SimConstants {
    public static final double stdDevX = 0.001;
    public static final double stdDevY = 0.001;
    public static final double stdDevHeading = 0.001;
    public static final double stdDevLeftVelocity = 0.1;
    public static final double stdDevRightVelocity = 0.1;
    public static final double stdDevLeftPosition = 0.005;
    public static final double stdDevRightPosition = 0.005;
  }

  public static final class DrivetrainConstants {
    public static final double gearRatio = 1 / ((11.0 / 60.0) * (20.0 / 28.0));
    public static final double estimatedMomentOfInertia = 4.5;
    public static final double estimatedMass = Units.lbsToKilograms(110);
    public static final double wheelRadius = 0.0635; // Units.inchesToMeters(5 / 2);
    public static final double trackWidth = Units.inchesToMeters(24);
    public static final DCMotor motorsPerSide = DCMotor.getNEO(2);

    public static final double distancePerPulse = gearRatio * wheelRadius * 2 * Math.PI;

    public static final DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(trackWidth);
    public static final double maxSpeed =
        Units.radiansPerSecondToRotationsPerMinute(motorsPerSide.freeSpeedRadPerSec)
            / 60
            * wheelRadius
            * 2
            * Math.PI
            / gearRatio;

    public static double kS = 0.2;
    public static double kV = 2.3;
    public static double kA = 0.01;
    public static double kP = 0.5;

    public static SimpleMotorFeedforward ramseteFF = new SimpleMotorFeedforward(kS, kV, kA);
  }
}
