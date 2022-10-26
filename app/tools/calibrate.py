from multiprocessing.sharedctypes import Value
import click
import json
import matplotlib.pyplot as plt
import numpy as np
import re
import serial

def parse_status(line):
    match = re.match("S1 (.*)", line)
    if match is None:
        print(f"Can't parse {line} as status")  
        return None, None, None
    else:    
        params = match.group(1)
        words = params.split(",")
        params = [float(w) for w in words]
        return params

@click.command()
@click.argument('port')
def main(port):
    ser = serial.Serial(port, 115200)

    ser.write(b"CAL \n")

    data = []

    power_on = False
    while True:
        line = ser.readline().decode('utf-8')
        encoder_angle, motor_angle, power = parse_status(line)
        if encoder_angle is None:
            continue
        print([encoder_angle, motor_angle, power])
        data.append([encoder_angle, motor_angle, power])
        if not power_on and power > 0.1: 
            power_on = True
        if power_on and power < 0.1:
            break
    
    
    print("Saving raw data to caldata.json")
    with open('caldata.json', 'w') as f:
        f.write(json.dumps(data))
    
    plt.figure()
    data = np.array(data)
    encoder_angle = data[:, 0]
    motor_angle = data[:, 1]
    plt.plot(motor_angle, encoder_angle)
    
    plt.show()

if __name__ == '__main__':
    main()