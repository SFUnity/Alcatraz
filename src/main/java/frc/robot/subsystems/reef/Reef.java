package frc.robot.subsystems.reef;

import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightHighAlgae;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightL1;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightL2;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightL3;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightLowAlgae;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightProcessor;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightSource;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.reef.elevator.Elevator;
import frc.robot.subsystems.reef.rollers.Rollers;
import frc.robot.util.VirtualSubsystem;

public class Reef extends VirtualSubsystem {
  private final Rollers rollers;
  private final Elevator elevator;

  public Reef(Rollers rollers, Elevator elevator) {
    this.rollers = rollers;
    this.elevator = elevator;
  }

  public void periodic() {

  }

  public boolean atDesiredHeight(double desiredHeight){
    return elevator.atDesiredHeight(desiredHeight);
  }
  public Command setCoralIntake(){
    return elevator
    .source()
    .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightSource)))
    .andThen(rollers.placeCoralAndDealgify());
  }

  public Command setScoreL1(){
    return elevator
    .source()
    .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightL1)))
    .andThen(rollers.placeCoralAndDealgify());
  }
  public Command setScoreL2(){
    return elevator
    .source()
    .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightL2)))
    .andThen(rollers.placeCoralAndDealgify());
  }
  public Command setScoreL3(){
    return elevator
    .source()
    .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightL3)))
    .andThen(rollers.placeCoralAndDealgify());
  }
  public Command setDealgaeLow(){
    return elevator
    .source()
    .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightLowAlgae)))
    .andThen(rollers.placeCoralAndDealgify());
  }
  public Command setDealgaeHigh(){
    return elevator
    .source()
    .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightHighAlgae)))
    .andThen(rollers.placeCoralAndDealgify());
  }

  public Command setScoreProcessor(){
    return elevator
    .source()
    .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightProcessor)))
    .andThen(rollers.scoreProcessor());
  }


}