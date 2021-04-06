package frc.robot;

import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Date;

public class Logger {
    public static final String LOGGER_KEY = "log";
    private static final int LOOKBACK_LENGTH = 5;
    private static final long LOOKBACK_TIME = 500L;

    private class LogEntry {
        public Date time;
        public String key;
        public String message;

        public LogEntry(String key, String message) {
            this.time = new Date();
            this.key = key;
            this.message = message;
        }

        public String toString() {
            StringBuilder sb = new StringBuilder();

            sb.append(this.time.toString());

            if (this.key != null) {
                sb.append(" [" + this.key + "]");
            }

            sb.append(" " + this.message);

            return sb.toString();
        }
    }
    
    private ArrayList<LogEntry> messages = new ArrayList<LogEntry>();

    private static Logger instance;

    private boolean shouldLogMessage(String messageString) {
        if (this.messages.size() <= LOOKBACK_LENGTH) return true;
        
        for (int i = this.messages.size() - 1 - LOOKBACK_LENGTH;
            i < this.messages.size();
            i++) {
            LogEntry msg = this.messages.get(i);
            Date earliestTime = new Date(System.currentTimeMillis() - LOOKBACK_TIME);

            if (earliestTime.before(msg.time) && messageString.equals(msg.message)) {
                return false;
            }
        }

        return true;
    }

    public static Logger getInstance() {
        if (instance == null) instance = new Logger();

        return instance;
    }

    private void logInternal(String key, String messageString) {
        if (!this.shouldLogMessage(messageString)) return;

        LogEntry msg = new LogEntry(key, messageString);
        String fullMessage = msg.toString();
        String sdCurrent = SmartDashboard.getString(LOGGER_KEY, "");

        this.messages.add(msg);

        SmartDashboard.putString(LOGGER_KEY, sdCurrent + fullMessage + "\n");
        System.out.println(fullMessage);
    }

    public static void log(String key, String messageString) {
        getInstance().logInternal(key, messageString);
    }

    public static void log(String messageString) {
        getInstance().logInternal(null, messageString);
    }
}
