import subprocess
import os
import csv
import time
import argparse

# usar doxygen

print("Debug start...")

parser = argparse.ArgumentParser(description="Capture logs from a stdout binary and save in .csv")
parser.add_argument("-b", "--bin", default="./build/examples/c/7_series/ahrs/7_series_ahrs_example_c", help="Binary file path to execute")
parser.add_argument("-o", "--out", required=True, help="Output file name to save in test/captures/")
args = parser.parse_args()

BINARY_FILE_PATH = args.bin#os.path.join("", args.bin)
THIS_FILE_ABS_PATH = os.path.dirname(os.path.abspath(__file__))
TEST_SCRIPT_PATH_REL_TO_PROJECT = "/test/python"
PROJECT_PATH = THIS_FILE_ABS_PATH.replace(TEST_SCRIPT_PATH_REL_TO_PROJECT, "")
BINARY_ABS_PATH = os.path.abspath(os.path.join(PROJECT_PATH, BINARY_FILE_PATH))

CSV_OUTPUT_FILE = os.path.abspath(os.path.join(PROJECT_PATH, "test/captures/", args.out))#"test/captures/logs.csv"))

print("BINARY_FILE_PATH:", BINARY_FILE_PATH)
print("THIS_FILE_ABS_PATH:", THIS_FILE_ABS_PATH)
print("PROJECT_PATH: ",PROJECT_PATH)
print("BINARY_ABS_PATH:", BINARY_ABS_PATH)
print("¿Existe el binario?:", os.path.exists(BINARY_ABS_PATH))
print("CSV_OUTPUT_FILE:", CSV_OUTPUT_FILE)

SUDO = "sudo"

with open(CSV_OUTPUT_FILE, "w", newline="") as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=';')
    spamwriter.writerow(["timestamp", "log", "tow", "data", "roll", "pitch", "yaw", "error"])
    
    process = subprocess.Popen(
            [SUDO, BINARY_ABS_PATH],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
    )

    for line in process.stdout:
        line = line.strip()
        print(line)

        tow = data = roll = pitch = yaw = error = None
 
        if "TOW" not in line:
            spamwriter.writerow([time.time(), line, None, None, None, None, None, None])
            csvfile.flush()
            continue

        try:
            part_tow = line.split("TOW =")[1].split()[0]
            tow     = float(part_tow)                
            
            if "Scaled Accel" in line:
                data = "Scaled"
            elif "Linear Accel" in line:
                data = "Linear"
            elif "Filter Euler Angles" in line:
                data = "filter"
            else:
                data = "unknown"

            inside = line.split("[")[1].split("]")[0]
            parts = inside.split(",")

            if len(parts) == 3:
                roll    = float(parts[0])
                pitch   = float(parts[1])
                yaw     = float(parts[2])
                
        except Exception as e:
            error = "error"
            print("Error parseando:", e)
 
        spamwriter.writerow([time.time(), line, tow, data, roll, pitch, yaw, error])
        csvfile.flush()   # guardar en disco inmediatamente
        
process.wait()

print("CSV guardado en:", CSV_OUTPUT_FILE)
