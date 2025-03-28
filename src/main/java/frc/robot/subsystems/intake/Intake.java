package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kRed);
  private final IntakeVisualizer setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kBlue);
  private double positionSetpoint = raisedAngle.get();

  private final LinearFilter currentFilter = LinearFilter.movingAverage(4);
  private double filteredCurrent;

  private boolean lowered = false;
  private boolean hasGP = false;
  private boolean startedIntaking = false;
  private boolean middleOfIntaking = false;
  public static boolean simHasGP = false;
  private boolean runningIceCream = false;

  private boolean lastGroundAlgae = groundAlgae.get();

  private final LoggedTunableNumber spikeCurrent =
      new LoggedTunableNumber("Intake/spikeCurrent", groundAlgae.get() ? 17 : 17);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    filteredCurrent = currentFilter.calculate(inputs.rollersCurrentAmps);
    Logger.recordOutput("Intake/filteredCurrent", filteredCurrent);
    Logger.recordOutput("Intake/startedIntaking", startedIntaking);
    lowered = inputs.pivotCurrentPositionDeg >= loweredAngle.get() / 2;

    if (groundAlgae.get()) {
      // There's a specific pattern in the current draw of the rollers that we're checking for here

      // Check that the pivot is lowered and not rising
      if ((inputs.pivotAppliedVolts <= 0.5 && lowered) || runningIceCream) {
        // Check if the current is high enough to be intaking
        if (filteredCurrent >= spikeCurrent.get() && !runningIceCream) {
          // check for start of intaking
          if (!startedIntaking && !hasGP) {
            startedIntaking = true;
          }
          // check for end of intaking
          if (middleOfIntaking && !hasGP) {
            hasGP = true;
            startedIntaking = false;
            middleOfIntaking = false;
          }
        }
        // check for dip in current
        if (filteredCurrent < spikeCurrent.get() && startedIntaking) {
          middleOfIntaking = true;
        }
        // check for massive current spike
        if (filteredCurrent >= 33) {
          hasGP = true;
          startedIntaking = false;
        }
      }
    } else {
      if (filteredCurrent > spikeCurrent.get() && inputs.pivotAppliedVolts <= 1.5 && lowered) {
        hasGP = true;
      }
    }

    if (lastGroundAlgae != groundAlgae.get()) {
      updateTunables();
      lastGroundAlgae = groundAlgae.get();
    }

    Logger.recordOutput("Intake/runningIceCream", runningIceCream);

    // Logs
    measuredVisualizer.update(Degrees.of(inputs.pivotCurrentPositionDeg));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("Intake/positionSetpoint", positionSetpoint);
    Util.logSubsystem(this, "Intake");

    Leds.getInstance().intakeGPHeld = GPHeld();
  }

  public Command resetGPHeld() {
    return Commands.runOnce(() -> hasGP = false);
  }

  private void lower() {
    positionSetpoint = loweredAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void setL1() {
    positionSetpoint = l1Angle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void raise() {
    positionSetpoint = raisedAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void rollersIn() {
    io.runRollers(rollersSpeedIn.get());
  }

  private void rollersOut() {
    io.runRollers(-rollersSpeedOut.get());
  }

  private void rollersStopOrHold() {
    double holdSpeed = groundAlgae.get() ? 0.15 : 0;
    io.runRollers(GPHeld() ? holdSpeed : 0);
  }

  public Command raiseAndStopOrHoldCmd() {
    return run(() -> {
          raise();
          rollersStopOrHold();
        })
        .withName("raise and stop");
  }

  public Command intakeCmd() {
    return run(() -> {
          lower();
          rollersIn();
        })
        .until(this::GPHeld)
        .withName("intake");
  }

  public Command runCurrentZeroing() {
    return this.run(() -> io.runPivot(-1.0))
        .until(() -> inputs.pivotCurrentAmps > 30.0)
        .finallyDo(() -> io.resetEncoder(0.0));
  }

  public Command poopCmd(BooleanSupplier shouldPlace) {
    final double highCurrent = groundAlgae.get() ? 10 : 15;
    final double lowCurrent = groundAlgae.get() ? 5 : 7;
    return Commands.waitUntil(() -> filteredCurrent > highCurrent)
        .andThen(
            Commands.waitUntil(() -> filteredCurrent < lowCurrent),
            Commands.runOnce(() -> hasGP = false))
        .raceWith(
            // The - number at the end is to build in some tolerance
            run(() -> setL1())
                .until(
                    () ->
                        inputs.pivotCurrentPositionDeg >= l1Angle.get() - .75
                                && shouldPlace.getAsBoolean()
                            || groundAlgae.get())
                .andThen(
                    run(() -> {
                          rollersOut();
                        })
                        .until(() -> !GPHeld())))
        .withName("poop");
  }

  public Command poopCmd() {
    return poopCmd(() -> true);
  }

  public Command iceCreamCmd() {
    return run(() -> {
          raise();
          rollersIn();
        })
        .beforeStarting(() -> runningIceCream = true)
        .finallyDo(() -> runningIceCream = false)
        .until(this::GPHeld)
        .withName("iceCream");
  }

  @AutoLogOutput
  public boolean GPHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasGP;
    }
    return hasGP;
  }
}
