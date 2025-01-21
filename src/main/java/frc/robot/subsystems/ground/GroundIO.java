package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIO {
  @AutoLog
  public static class GroundIOInputs {
    public Angle pivotCurrentPosition = Rotations.zero();
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double rollersAppliedVolts = 0.0;
    public double rollersCurrentAmps = 0.0;
  }

  default void updateInputs(GroundIOInputs inputs) {}

  default void runGroundRollers(double percentOutput) {}

  default void setPivotPosition(Angle angle) {}

  default void setPID(double p) {}

  default void stop() {}
}