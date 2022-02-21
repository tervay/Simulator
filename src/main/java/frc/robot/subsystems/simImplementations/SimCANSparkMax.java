package frc.robot.subsystems.simImplementations;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimCANSparkMax extends CANSparkMax {

  private SimDeviceSim simDeviceSim;

  public SimCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
    simDeviceSim = new SimDeviceSim("SPARK MAX [" + deviceId + "]");
  }

  public SimDeviceSim getSimDeviceSim() {
    return simDeviceSim;
  }

  public void setVelocity(double velocity) {
    simDeviceSim.getDouble("Velocity").set(velocity);
  }
}
