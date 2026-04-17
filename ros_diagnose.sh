#!/bin/bash
set -e
LOG=~/ros_diagnose_$(date +%s).log
echo "Diagnostics log: $LOG"
echo "=== WHICH / ROS VERSION ===" > "$LOG"
which roscore >> "$LOG" 2>&1 || true
rosversion -d >> "$LOG" 2>&1 || true

echo "\n=== ROS ENV VARS ===" >> "$LOG"
env | egrep 'ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP|ROS_PACKAGE_PATH|ROS_ROOT' >> "$LOG" 2>&1 || true

echo "\n=== PS (roscore/rosmaster/rosout) ===" >> "$LOG"
ps aux | egrep '(roscore|rosmaster|rosout)' | egrep -v egrep >> "$LOG" 2>&1 || true

echo "\n=== PORT 11311 (ss) ===" >> "$LOG"
ss -ltnp | grep 11311 >> "$LOG" 2>&1 || true

echo "\n=== TRY STARTING roscore (background, will stop after capture) ===" >> "$LOG"
# start roscore in background and capture output
roscore > /tmp/roscore_stdout.log 2>&1 &
RC_PID=$!
echo "roscore pid: $RC_PID" >> "$LOG"
sleep 3

echo "\n--- /tmp/roscore_stdout.log (tail) ---" >> "$LOG"
tail -n 200 /tmp/roscore_stdout.log >> "$LOG" 2>&1 || true

# give it a moment then kill if still running
sleep 1
if ps -p $RC_PID >/dev/null 2>&1; then
  kill $RC_PID >/dev/null 2>&1 || true
  sleep 1
fi

echo "\n=== ~/.ros/log listing ===" >> "$LOG"
ls -lt ~/.ros/log >> "$LOG" 2>&1 || true

echo "\nDiagnostics finished. Log file: $LOG" >> "$LOG"
cat "$LOG"
