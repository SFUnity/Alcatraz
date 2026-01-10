package frc.robot.subsystems.funnel;

import static frc.robot.subsystems.funnel.FunnelConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.carriage.CarriageConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  public Funnel(FunnelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);
  }

  public Command runRollers() {
    return run(() -> io.runVolts(rollerSpeedVolts.get()));
  }

  public Command eject() {
    return run(() -> io.runVolts(-rollerSpeedVolts.get()))
        .withTimeout(CarriageConstants.ejectCoralTime.get());
  }

  public Command stop() {
    return run(() -> io.runVolts(0));
  }

  public Command defaultCmd(BooleanSupplier coralHeld, BooleanSupplier intakingAlgae) {
    return run(
        () -> {
          if (coralHeld.getAsBoolean() || intakingAlgae.getAsBoolean()) {
            io.runVolts(0);
          } else {
            io.runVolts(rollerSpeedVolts.get());
          }
        });
  }
}
