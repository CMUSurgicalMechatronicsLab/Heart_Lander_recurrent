'''
Author: Yi Wang
Date: Aug-2024
'''
from EM_tracker.atc3dg_types import *
from Cerberus_system import *
from ST3215 import ST3215 
from time import sleep


def main():
    """Full workflow for HeartPrinter (Cerberus)"""

    Cerberus_robot = Cerberus_system()

    Cerberus_robot.set_delay(10)  # set EM tracker collection delay in ms

    # Initialize points array for registration
    Cerberus_reg_records_transform = []

    # Initialize EM tracker with angle align
    print("To initialize the EM tracker, the injector head with the EM sensor inside must be placed flat on a table in front of the transmitter's front hemisphere.")
    print("Align the distal end of the injector head with the +'ve y-axis.")
    val = input('Please confirm this is true: "y/n": ')
    while val.lower() != 'y':
        val = input('Please confirm this is true: "y/n": ')

    print("\nInitializing all Tracker components...")
    Cerberus_robot.initialize_tracker()
    print("Done initializing TrakStar")

    print("\nInitializing DAQ (USB-1208LS)..")
    Cerberus_robot.initialize_daq()
    print("Done initializing DAQ")

    print("\nInitializing ST3215 motors via ESP32..")
    anterior_motor = ST3215(port='COM1', baudrate=115200)
    inferior_motor = ST3215(port='COM2', baudrate=115200)
    posterior_motor = ST3215(port='COM3', baudrate=115200)
    anterior_motor.enable()
    inferior_motor.enable()
    posterior_motor.enable()
    print("Done initializing motors")

    print("\nAre the robot components near the tabletop setup? If not, they will be as a homing.")
    val = input('y/n: ')
    if val.lower() == 'n':
        duration = int(input('Enter the amount of seconds to pull the components in: '))
        print("\nNow the robot components will be pulled towards the tabletop setup")
        anterior_motor.set_angle(0)
        inferior_motor.set_angle(0)
        posterior_motor.set_angle(0)
        sleep(duration)

    # Introducer Mechanism now positions the robot on heart
    print("\nFor the introducer mechanism, cable slack will first be given to the posterior base")
    sleep(5)
    posterior_motor.set_angle(90)  # Assuming 90 degrees gives slack
    print("\nNow attach the suction line to the posterior base, position it, and activate suction.")
    val = input('Please confirm this is done: "y/n": ')
    while val.lower() != 'y':
        val = input('Please confirm this is done: "y/n": ')

    print("\nNow cable slack will be given to the anterior and inferior bases to bring them close to the subxiphoid port")
    sleep(5)
    anterior_motor.set_angle(90)
    inferior_motor.set_angle(90)
    print("\nSome more slack will be given to the anterior base for positioning.")
    sleep(3)
    anterior_motor.set_angle(120)  # More slack to the anterior base
    print("\nNow attach the suction line to the anterior base, position it, and activate suction.")
    val = input('Please confirm this is done: "y/n": ')
    while val.lower() != 'y':
        val = input('Please confirm this is done: "y/n": ')

    print("\nNow position the inferior suction base and injector head")
    sleep(3)
    inferior_motor.set_angle(120)  # Slack for the inferior base
    val = input('Please confirm this is done: "y/n": ')
    while val.lower() != 'y':
        val = input('Please confirm this is done: "y/n": ')

    # Determine the position of the inferior suction base
    print("\nNow determining the position of the inferior base relative to the transmitter...")
    inferior_position, Cerberus_reg_inferior_records_transform = Cerberus_robot.inferior_base()
    print("x_inferior:", inferior_position[0])
    print("y_inferior:", inferior_position[1])
    print("z_inferior:", inferior_position[2])

    # Determine the positions of the superior suction bases & collect points for registration
    print("\nNow determining the points for registration and the position of the superior bases relative to the transmitter...")
    sleep(3)
    Cerberus_reg_trajectory_records_transform, Cerberus_reg_anterior_records_transform, Cerberus_reg_posterior_records_transform, anterior_position, posterior_position = Cerberus_robot.collect_trajectory()

    print("\nx_anterior:", anterior_position[0])
    print("y_anterior:", anterior_position[1])
    print("z_anterior:", anterior_position[2])

    print("\nx_posterior:", posterior_position[0])
    print("y_posterior:", posterior_position[1])
    print("z_posterior:", posterior_position[2])

    # Compile array of points for registration
    Cerberus_reg_records_transform.append(Cerberus_reg_inferior_records_transform)
    Cerberus_reg_records_transform.append(Cerberus_reg_trajectory_records_transform)
    Cerberus_reg_records_transform.append(Cerberus_reg_anterior_records_transform)
    Cerberus_reg_records_transform.append(Cerberus_reg_posterior_records_transform)
    Cerberus_reg_records_transform.append([inferior_position, anterior_position, posterior_position])

    tot_length = (len(Cerberus_reg_inferior_records_transform) +
                  len(Cerberus_reg_trajectory_records_transform) +
                  len(Cerberus_reg_anterior_records_transform) +
                  len(Cerberus_reg_posterior_records_transform) + 3)

    Cerberus_reg_records_transform_final = np.empty([tot_length, 6])  # initialize numpy array for data
    i = 0
    for location in Cerberus_reg_records_transform:
        for data in location:
            Cerberus_reg_records_transform_final[i, 0] = data[0]
            Cerberus_reg_records_transform_final[i, 1] = data[1]
            Cerberus_reg_records_transform_final[i, 2] = data[2]
            Cerberus_reg_records_transform_final[i, 3] = data[3]
            Cerberus_reg_records_transform_final[i, 4] = data[4]
            Cerberus_reg_records_transform_final[i, 5] = data[5]
            i += 1

    # Write registration points to txt file which has the transformed (x,y,z) points
    # Last 3 rows are suction base locations
    np.savetxt("Injector_Head_Registration_points.txt", Cerberus_reg_records_transform_final, fmt='%f')

    # Disable motors and close the serial connections
    anterior_motor.disable()
    inferior_motor.disable()
    posterior_motor.disable()
    anterior_motor.close()
    inferior_motor.close()
    posterior_motor.close()


if __name__ == '__main__':
    main()
