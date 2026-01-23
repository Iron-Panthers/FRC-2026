package frc.robot.subsystems.elastic_updater;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElasticUpdater extends SubsystemBase {

    @Override
    public void periodic() {
        
        double matchTime;
        Logger.recordOutput("MatchTime", matchTime = DriverStation.getMatchTime());

        if (DriverStation.isAutonomous()) {
            
            updateMatchData((matchTime >= 0 && matchTime <= 30) ? "Auto" : "Invalid Timeframe");

        } else if (DriverStation.isTeleop()) {

            if (matchTime > 140) {
                updateMatchData("Invalid Timeframe");
            } else if (matchTime > 130) {
                updateMatchData("Transition Shift");
            } else if (matchTime > 105) {
                updateMatchData("Shift 1");
            } else if (matchTime > 80) {
                updateMatchData("Shift 2");
            } else if (matchTime > 55) {
                updateMatchData("Shift 3");
            } else if (matchTime > 30) {
                updateMatchData("Shift 4");
            } else if (matchTime > 0) {
                updateMatchData("End Game");
            } else {
                updateMatchData("Invalid Timeframe");
            }
        } else {
            updateMatchData("Invalid Timeframe");
        }
    }

    private void updateMatchData(String timeframe) {
        Logger.recordOutput("MatchTimeframe", timeframe);

        boolean redHubActive = !(timeframe == "Invalid Timeframe");
        boolean blueHubActive = !(timeframe == "Invalid Timeframe");

        if (timeframe.startsWith("Shift")) {
            // DriverStation.getGameSpecificMessage returns the team that is active on shifts 2 and 4. https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
            // If it doesn't get set to anything (i.e. it's not a match currently) then I set it so the default is red goes first
            redHubActive = DriverStation.getGameSpecificMessage() == "R";
            redHubActive = (redHubActive) ^ (timeframe == "Shift 1" || timeframe == "Shift 3");
            blueHubActive = !redHubActive;
        }

        Logger.recordOutput("RedHubActive", redHubActive);
        Logger.recordOutput("BlueHubActive", blueHubActive);
    }
}