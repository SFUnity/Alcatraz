package frc.robot.subsystems.carriage;

import static frc.robot.subsystems.carriage.CarriageConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Carriage extends SubsystemBase {
  private final CarriageIO io;
  private final CarrageIOInputsAutoLogged inputs = new CarrageIOInputsAutoLogged();

  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double filteredVelocity;
  private double filteredStatorCurrent;

  public static boolean simHasCoral = false;
  public static boolean simHasAlgae = false;
  public static boolean simBeamBreak = false;

  public boolean coralPassed = false;
  private boolean realCoralHeld = false;
  private boolean realAlgaeHeld = false;
  private boolean fullCoralHeld = false;

  private final Timer beambreakTimer = new Timer();
  private static final LoggedTunableNumber beambreakDelay =
      new LoggedTunableNumber("Carriage/beambreakDelay", 0.3);
  private final Timer fullCoralHeldTimer = new Timer();
  private static final LoggedTunableNumber fullCoralHeldDelay =
      new LoggedTunableNumber("Carriage/fullCoralHeldDelay", 0.35);

  private static final LoggedTunableNumber backupForL3Rots =
      new LoggedTunableNumber("Carriage/Backup for L3 Rots", 15);

  public static boolean coralInDanger = false;
  private boolean lastShouldBrake = false;

  public Carriage(CarriageIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Carriage", inputs);

    updateCoralStatus();

    // Check for brake mode
    boolean shouldBrake = !DriverStation.isDisabled() || beamBreak();
    if (shouldBrake != lastShouldBrake) {
      io.setBrakeMode(shouldBrake);
      lastShouldBrake = shouldBrake;
      if (DriverStation.isDisabled()) {
        realCoralHeld = shouldBrake;
      }
    }

    // Check for algae held
    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.velocityRotsPerSec));
    filteredStatorCurrent = currentFilter.calculate(inputs.currentAmps);

    if (filteredVelocity <= algaeVelocityThreshold.get()
        && filteredStatorCurrent >= algaeCurrentThreshold.get()) {
      realAlgaeHeld = true;
    }

    // Leds
    Leds.getInstance().coralPassed = coralPassed;
    Leds.getInstance().carriageAlgaeHeld = algaeHeld();

    // Logging
    Logger.recordOutput("Carriage/filteredVelocity", filteredVelocity);
    Logger.recordOutput("Carriage/filteredStatorCurrent", filteredStatorCurrent);
    Logger.recordOutput("Carriage/coralPassed", coralPassed);
    Logger.recordOutput("Carriage/coralInDanger", coralInDanger);

    Util.logSubsystem(this, "Carriage");
  }

  public void updateCoralStatus() {
    if (!beamBreak() && !coralPassed) {
      realCoralHeld = false;
    } else if (!beamBreak() && coralPassed) {
      realCoralHeld = true;
    } else if (beamBreak() && !coralPassed && !realCoralHeld) {
      if (beambreakTimer.hasElapsed(beambreakDelay.get())) {
        coralPassed = true;
      }
    } 
    if (!beamBreak() || coralPassed || realCoralHeld) {
      beambreakTimer.restart();
    }
    if (beamBreak() && realCoralHeld) {
      if (fullCoralHeldTimer.hasElapsed(fullCoralHeldDelay.get())) {
        fullCoralHeld = true;
      }
    } else {
      fullCoralHeld = false;
      fullCoralHeldTimer.restart();
    }
  }

  @AutoLogOutput
  public boolean coralHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasCoral || realCoralHeld;
    }
    return realCoralHeld;
  }

  @AutoLogOutput
  public boolean fullCoralHeld() {
    return fullCoralHeld;
  }

  public Command resetHeld() {
    return Commands.runOnce(
        () -> {
          realCoralHeld = false;
          coralPassed = false;
          realAlgaeHeld = false;
        });
  }

  @AutoLogOutput
  public boolean algaeHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasAlgae;
    }
    return realAlgaeHeld;
  }

  @AutoLogOutput
  public boolean beamBreak() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simBeamBreak;
    }
    return inputs.beamBreak;
  }

  public Command stopOrHold() {
    return run(() -> {
          if (!beamBreak() && realCoralHeld) {
            if (inputs.currentAmps < 5) {
              realAlgaeHeld = false;
            }
            io.runVolts(-holdSpeedVolts.get());
          } 
          else {
            io.runVolts(algaeHeld() ? holdSpeedVolts.get() : 0);
          }
        })
        .onlyWhile(() -> coralHeld() || algaeHeld())
        .withName("stop");
  }

  public Command backUpForL3() {
    return run(() -> io.runVolts(backwardsIntakeSpeedVolts.get()))
        .finallyDo(() -> io.runVolts(0))
        .beforeStarting(() -> io.resetEncoder())
        .until(() -> inputs.positionRots >= backupForL3Rots.get())
        .withName("backUpForL3");
  }

  private Command privatePlaceCoral(LoggedTunableNumber placeSpeed) {
    return run(() -> io.runVolts(placeSpeed.get()))
        .until(() -> !beamBreak())
        .andThen(() -> realCoralHeld = false)
        .andThen(() -> coralPassed = false)
        .withName("placeCoral");
  }

  public Command placeCoral() {
    return privatePlaceCoral(placeSpeedVolts);
  }

  public Command placeL1Coral() {
    return privatePlaceCoral(placeL1SpeedVolts);
  }

  public Command highDealgify() {
    return run(() -> io.runVolts(highDealgifyingSpeedVolts.get())).withName("highDealgify");
  }

  public Command lowDealgify() {
    return run(() -> io.runVolts(lowDealgifyingSpeedVolts.get()))
        .until(() -> algaeHeld())
        .withName("lowDealgify");
  }

  public Command intakeCoral() {
    return Commands.either(
            run(() -> io.runVolts(placeSpeedVolts.get())),
            run(() -> io.runVolts(intakingSpeedVolts.get()))
                .until(() -> coralPassed)
                .andThen(
                    run(() -> io.runVolts(-backwardsIntakeSpeedVolts.get())).until(() -> !beamBreak()),
                    run(() -> io.runVolts(backwardsIntakeSpeedVolts.get()))
                        .until(() -> beamBreak()))
                .onlyIf(() -> !coralHeld()),
            () -> coralInDanger)
        .deadlineFor(
            Commands.runEnd(
                () -> Leds.getInstance().intakingActivated = true,
                () -> Leds.getInstance().intakingActivated = false))
        .withName("intake coral");
  }

  public Command scoreProcessor() {
    return run(() -> io.runVolts(processorSpeedVolts.get()))
        .withTimeout(.2)
        .finallyDo(() -> realAlgaeHeld = false)
        .withName("scoreProcessor");
  }

  public Command ejectAlgae() {
    return run(() -> io.runVolts(ejectSpeedVolts.get()))
        .withTimeout(.2)
        .finallyDo(() -> realAlgaeHeld = false)
        .withName("ejectAlgae");
  }
}
