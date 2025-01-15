package frc.robot.subsystems.reef.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private final DigitalInput beamBreak = new DigitalInput(RollersConstants.beamBreakNumber);

  public Rollers(RollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  private void runRollers(double volts) {
    io.runVolts(volts);
  }

  private void stopRollers() {
    io.stop();
  }

  public boolean coralHeld() {
    return !beamBreak.get();
  }

  public boolean algaeHeld() {
    double filteredVelocity = velocityFilter.calculate(Math.abs(inputs.velocityRotsPerSec));
    double filteredStatorCurrent = currentFilter.calculate(inputs.currentAmps);
    return (filteredVelocity <= RollersConstants.algaeVelocityThreshold
            && (filteredStatorCurrent >= RollersConstants.algaeCurrentThreshold)
        || filteredStatorCurrent <= -2);
  }

  public Command placeCoralAndHighAlgae() {
    return run(() -> {
          io.runMotorStraight();
          runRollers(RollersConstants.rollersPlaceSpeed);
        })
        .withName("placeCoralRollers");
  }

  public Command lowDeAlgaefy() {
    return run(() -> {
          io.runMotorStraight();
          runRollers(RollersConstants.rollersDealgifyingSpeed);
        })
        .until(() -> algaeHeld())
        .andThen(
            run(
                () -> {
                  stopRollers();
                }))
        .withName("deAlgaefy");
  }

  public Command intakeCoral() {
    return run(() -> {
          io.runMotorStraight();
          runRollers(RollersConstants.rollersIntakingSpeed);
        })
        .until(() -> coralHeld())
        .andThen(
            run(
                () -> {
                  stopRollers();
                }))
        .withName("intakeCoralRollers");
  }

  public Command scoreProcessor() {
    return run(() -> {
          io.reverseMotor();
          runRollers(RollersConstants.rollersIntakingSpeed);
        })
        .withName("scoreProcessor");
  }
  // TODO: add "until when 2m distance sensor gets set up"
}
