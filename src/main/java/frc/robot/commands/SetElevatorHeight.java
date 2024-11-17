// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorHeight extends Command {

    private final Elevator m_elevator;
    private final double m_heightMeters;

    /** Creates a new SetElevator. */
    public SetElevatorHeight(Elevator elevator, double heightMeters) {
        m_elevator = elevator;
        m_heightMeters = heightMeters;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_elevator);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setGoal(m_heightMeters);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_elevator.getHeight() - m_heightMeters) < Elevator.HEIGHT_TOLERANCE;
    }
}
