#!/bin/bash

# Log files
ros_log="ros_processes.log"
java_log="java_processes.log"

# Interval between logging in seconds
interval=1

# Total duration for logging in seconds (2 minutes = 120 seconds)
duration=120

# Get the number of CPUs
total_cpus=$(nproc)

# Function to log processes
log_processes() {
    process_name=$1
    log_file=$2
    iteration=$3

    # Get total CPU and memory usage of the processes
    usage=$(ps aux | grep $process_name | grep -v grep | awk '{cpu+=$3; mem+=$6} END {print cpu, mem}')
    cpu_usage=$(echo $usage | awk '{print $1}')
    mem_usage_kb=$(echo $usage | awk '{print $2}')

    # Convert memory usage to MB
    mem_usage_mb=$(echo "scale=2; $mem_usage_kb / 1024" | bc)

    # Calculate relative CPU usage
    relative_cpu=$(echo "scale=2; $cpu_usage / $total_cpus" | bc)

    # Log the information
    echo "iteration: $iteration ; CPU: $relative_cpu% ; MEM: $mem_usage_mb MB" >> $log_file
}

# Create or clear the log files
echo "ROS Processes Log" > $ros_log
echo "Java Processes Log" > $java_log

# Calculate the number of iterations
iterations=$((duration / interval))

# Loop to log the processes every interval for the total duration
for ((i = 1; i <= iterations; i++))
do
    log_processes "ros" $ros_log $i
    log_processes "java" $java_log $i
    sleep $interval
done

