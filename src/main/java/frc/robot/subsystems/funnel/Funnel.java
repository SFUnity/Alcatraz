package frc.robot.subsystems.funnel;

import static frc.robot.subsystems.funnel.FunnelConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  public Funnel(FunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command runRollers() {
    return run(() -> io.runVolts(rollerSpeedVolts.get()));
  }

  public Command eject() {
    return run(() -> io.runVolts(-rollerSpeedVolts.get()));
  }

  public Command stop() {
    return run(() -> io.runVolts(0));
  }
}
