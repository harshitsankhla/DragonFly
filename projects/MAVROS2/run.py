import json
import mavros2


if __name__ == '__main__':
    dragonfly = mavros2.Vehicle(json.load(open('config.json')))

    while True:
        print("-------------------------------------")
        print("-----------CONTROL CENTER------------")
        print("-------------------------------------")
        print("1. Arm")
        print("2. Takeoff")
        print("3. Land")
        print("4. Disarm")
        print("5. Offboard")

        val = int(input("Enter your choice - "))
        if val == 1:
            dragonfly.arm()
        elif val == 2:
            dragonfly.takeoff()
        elif val == 3:
            dragonfly.land()
        elif val == 4:
            dragonfly.disarm()
        elif val == 5:
            dragonfly.offboard()
        else:
            print("GoodBye !")
            break

