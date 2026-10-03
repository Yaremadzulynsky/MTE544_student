from math import atan2, asin, sqrt
import csv
import json

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w', newline='') as file:
            csv.writer(file).writerow(headers)


    def log_values(self, values_list):

        with open(self.filename, 'a', newline='') as file:
            # TODONE Part 5: Write the values from the list to the file
            # Quote array fields as one CSV cell. Keep invalid scan readings for
            # later filtering, and preserve integer nanosecond timestamps.
            csv.writer(file).writerow([
                json.dumps(value) if isinstance(value, (list, tuple)) else value
                for value in values_list
            ])
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        table=[]
        with open(self.filename, 'r', newline='') as file:
            # Skip the header line
            reader=csv.reader(file)
            headers=[value.strip() for value in next(reader, []) if value.strip()]
            # Read every data row, including the first, and decode scan arrays.
            for values in reader:
                if not values:
                    continue
                row=[]
                for header, value in zip(headers, values):
                    value=value.strip()
                    if header == 'ranges':
                        row.append(json.loads(value))
                    elif header == 'stamp':
                        row.append(int(value))
                    else:
                        row.append(float(value))
                table.append(row)
        
        return headers, table


# TODONE Part 5: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w=quat # just unpack yaw
    yaw=atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    return yaw

