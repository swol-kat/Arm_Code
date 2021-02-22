import odrive
import odrive.enums
from arm.joint import Joint

odrives = odrive.find_any(find_multiple = int(input("how many odrives? ")))

jointList = []

for od in odrives:
    joint0 = Joint(od.axis0, 1.0)
    joint0.motor_configuration(4)
    joint1 = Joint(od.axis1, 1.0)
    joint1.motor_configuration(5)
    od.save_configuration()
    print(od.serial_number)
    

    