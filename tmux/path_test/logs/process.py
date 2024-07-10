#!/usr/bin/python3
import re

# Define the base name pattern for the log files
base_name_pattern = '{}_heading.txt'

# Define the output file name pattern
output_name_pattern = '{}_heading_processed.txt'

# Define a regex pattern to extract the necessary information
pattern = re.compile(
    r'secs: (\d+)\s+nsecs: (\d+)[\s\S]+?value: ([\-.\d]+)',
    re.MULTILINE
)

# Iterate over the range of files (1 to 5)
for i in range(1, 6):
    # Generate the log file name and output file name
    log_file_path = base_name_pattern.format(i)
    output_file_path = output_name_pattern.format(i)

    # Read the log file content
    with open(log_file_path, 'r') as log_file:
        log_data = log_file.read()

    # Find all matches in the log data
    matches = pattern.findall(log_data)

    # Open the output file
    with open(output_file_path, 'w') as output_file:
        # Write the matches to the file in the desired format
        for match in matches:
            secs, nsecs, heading = match
            output_file.write(f"sec : {secs} ; nsec : {nsecs} ; Heading : {heading}\n")

    print(f"Processing complete. The output file is '{output_file_path}'.")

print("All files processed.")
