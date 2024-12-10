// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class ChoreoVSPathplannerAutoTest extends SequentialCommandGroup {
    // private Pose2d m_initPose;

    /** Creates a new SimpleAuto. */
    public ChoreoVSPathplannerAutoTest(DriveTrain driveTrain) {
        //NOTE: TOGGLE TO FALSE TO USE PATH PLANNER PATH
        Boolean choreoPathEnabled = true;

        PathPlannerPath startPath = null;
        if (choreoPathEnabled) {
        try {
            startPath = PathPlannerPath.fromChoreoTrajectory("ChoreoPath C1 to shoot");
        } catch (FileVersionException e) {
            DriverStation.reportError("Choreo path error", true);
        } catch (IOException e) {
            DriverStation.reportError("Choreo path error", true);
        } catch (ParseException e) {
            DriverStation.reportError("Choreo path error", true);
        }
        } else {
            startPath = DriveTrain.loadPath("PathPlannerPath C1 to shoot");
        }
        // m_initPose = startPath.getStartingDifferentialPose();
        addCommands(driveTrain.followPath(startPath));
    }
}
