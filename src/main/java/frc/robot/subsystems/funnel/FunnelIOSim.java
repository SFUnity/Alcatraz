package frc.robot.subsystems.funnel;

public class FunnelIOSim implements FunnelIO {
  private double appliedVolts = 0.0;

  public FunnelIOSim() {}

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
