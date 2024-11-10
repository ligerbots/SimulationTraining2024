// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand = null;
    private boolean m_prevIsRedAlliance = true;

    private RobotContainer m_robotContainer;
    
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {
        if (isSimulation()) {
            // YAGSL bug fix (Nov 2024)
            // The simulation needs to be told the drive speed every loop, even when disabled 
            m_robotContainer.getDriveTrain().drive(0, 0, 0, false);
        }

        boolean isRedAlliance = FieldConstants.isRedAlliance();
        if (m_autonomousCommand == null || isRedAlliance != m_prevIsRedAlliance) {
            m_autonomousCommand = m_robotContainer.getAutonomousCommand();
            m_robotContainer.getDriveTrain().setPose(m_robotContainer.getInitialPose());
            m_prevIsRedAlliance = isRedAlliance;
        }
    }
    
    @Override
    public void disabledExit() {}
    
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void autonomousExit() {}
    
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void teleopExit() {}
    
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override
    public void testPeriodic() {}
    
    @Override
    public void testExit() {}
}
