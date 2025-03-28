package frc.robot.subsystems.funnel;

import static frc.robot.subsystems.funnel.FunnelConstants.*;
import static frc.robot.util.SparkUtil.configureSpark;
import static frc.robot.util.SparkUtil.sparkConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class FunnelOSparkMax implements FunnelIO {
  private final SparkMax rollerMotor = new SparkMax(funnelMotorID, MotorType.kBrushless);
  private final RelativeEncoder encoder = rollerMotor.getEncoder();
  private final SparkMaxConfig config = sparkConfig(inverted, funnelMotorID);

  public FunnelOSparkMax() {
    configureSpark(rollerMotor, config, true);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.currentAmps = rollerMotor.getOutputCurrent();
  }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }
}
