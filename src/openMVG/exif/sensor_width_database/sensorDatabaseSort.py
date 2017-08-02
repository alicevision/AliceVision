import argparse

# command line
parser = argparse.ArgumentParser(description='Sort sensor width camera database')
parser.add_argument('-i', '--input', metavar='inputSensorDatabase.txt',required=True , type=str, 
					help='input sensor width camera database')
parser.add_argument('-o', '--output', metavar='outputSensorDatabase.txt', required=True , type=str,
                    help='sorted output sensor width camera database')

args = parser.parse_args()

# read
file = open(args.input, "r")
database = file.readlines()
file.close()

# process
sensors = [entry.split(';')  for entry in database]
sensors.sort()
outDatabase = ""
for entry in sensors:
	outDatabase += ';'.join(entry)

# write
file = open(args.output, "w")
file.write(outDatabase)
file.close()

