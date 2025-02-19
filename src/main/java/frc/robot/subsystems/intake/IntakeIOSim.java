package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constantsGlobal.Constants;

public class IntakeIOSim implements IntakeIO {

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          50,
          0.5,
          armLengthMeters,
          minAngleRads,
          maxAngleRads,
          false,
          maxAngleRads);

  private final PIDController controller;
  private double pivotAppliedVolts = 0.0;
  private double rollersAppliedVolts = 0.0;

  public IntakeIOSim() {
    controller = new PIDController(kP.get(), 0.0, 0.0);
    sim.setState(maxAngleRads, 0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    inputs.pivotCurrentPositionDeg = Units.radiansToDegrees(sim.getAngleRads());
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = sim.getCurrentDrawAmps();

    inputs.rollersAppliedVolts = rollersAppliedVolts;

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void runRollers(double volts) {
    rollersAppliedVolts = volts;
  }

  @Override
  public void setPivotPosition(double angle) {
    double volts = controller.calculate(sim.getAngleRads(), Units.degreesToRadians(angle));
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(pivotAppliedVolts);
  }
}
