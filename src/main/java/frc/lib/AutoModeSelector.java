package frc.lib;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/**
 * Builds an Auto Mode Chooser by merging the
 * PathPlanner generated Autos with manually specified auto commands
 */
public class AutoModeSelector
{
    private final SendableChooser<Command> modeSelector;

    private final String defaultModeKey = "None";
    private final Command defaultMode = new WaitCommand(15.0);

    private Optional<Command> currentMode;

    public AutoModeSelector()
    {
        // build the chooser with the Path Planner Autos
        // modeSelector = AutoBuilder.buildAutoChooser();
        // OR use our builder which filters on PathPlanner folders
        modeSelector = buildAutoChooser(""); // empty folder holds our current autos

        // Add any manually created auto commands here
        // add("Test Something", new ModeTestSomething());
        // add("Test Driving", new ModeTestDriving());
        // add("1 close", new Mode1Close());
        // add("Test", new Test());

        setDefault(defaultModeKey, defaultMode);

        // push the chooser to the dashboard
        SmartDashboard.putData("Auto Mode", modeSelector);
    }

    /**
     * Add a new auto mode option
     */
    public void add(String name, Command command)
    {
        modeSelector.addOption(name, command);
    }

    /**
     * Set the default auto mode
     */
    public void setDefault(String name, Command command)
    {
        modeSelector.setDefaultOption(name, command);
    }

    /**
     * Call to get the latest mode from the Dashboard
     */
    public boolean update()
    {
        Command desiredMode = modeSelector.getSelected();

        if (desiredMode == null)
        {
            desiredMode = defaultMode;
        }

        if (!currentMode.equals(Optional.of(desiredMode)))
        {
            System.out.println("Auto: " + desiredMode);
            currentMode = Optional.of(desiredMode);
            return true;
        }

        return false;
    }

    /**
     * Reset the current mode to the default mode
     */
    public void reset()
    {
        currentMode = Optional.of(defaultMode);
    }

    /**
     * Get the currently selected Auto Mode
     */
    public Optional<Command> getAutoMode()
    {
        return currentMode;
    }

    public ArrayList<Pose2d> getPathPreview()
    {
        ArrayList<Pose2d> preview = new ArrayList<>();

        if (getAutoMode().isPresent())
        {
            String name = getAutoMode().get().getName();

            try
            {
                List<PathPlannerPath> group = PathPlannerAuto.getPathGroupFromAutoFile(name);

                for (PathPlannerPath p : group)
                {
                    if (RobotContainer.isOnRedAlliance())
                    {
                        preview.addAll(p.flipPath().getPathPoses());
                    }
                    else
                    {
                        preview.addAll(p.getPathPoses());
                    }
                }
            } catch (IOException | ParseException | RuntimeException e)
            {
            }
        }

        return preview;
    }

    /**
     * Based on PathPlanner's AutoBuilder, will build the Auto but only if it's within the specified PathPlanner folder.
     * Not optimized, reads the auto file twice
     * 
     * @param folderToUse
     * @return
     */
    private static SendableChooser<Command> buildAutoChooser(String folderToUse)
    {
        if (!AutoBuilder.isConfigured())
        {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }
        SendableChooser<Command> chooser = new SendableChooser<>();
        List<String> autoNames = AutoBuilder.getAllAutoNames();

        chooser.setDefaultOption("None", Commands.none());
        for (String autoName : autoNames)
        {
            // only add if it's within the folder
            if (getFolderFromAutoPath(autoName).equals(folderToUse))
            {
                PathPlannerAuto auto = new PathPlannerAuto(autoName);
                chooser.addOption(auto.getName(), auto);
            }
        }

        return chooser;
    }

    /**
     * Get the Path Planner folder for an auto routine
     * 
     * @param autoName
     * @return
     */
    private static String getFolderFromAutoPath(String autoName)
    {
        String folder = "";

        try (BufferedReader br = new BufferedReader(
                new FileReader(
                        new File(
                                Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto"))))
        {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null)
            {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            Object folderStr = json.get("folder");

            if (folderStr != null)
                folder = folderStr.toString();
        } catch (Exception e)
        {
            System.out.println(e);
        }
        return folder;
    }
}
