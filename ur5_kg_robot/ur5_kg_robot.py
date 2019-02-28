import time
import copy
import serial
from math import pi
import numpy
import json
import socket

import waypoints as wp
import specialised_kg_robot_example as kgrs # rename for your project

def main():
    print("------------Configuring Burt-------------\r\n")
    burt = 0
    burt = kgrs.specialised_kg_robot(port=30010,db_host="192.168.1.10",ee_port="COM38",test=True)
    print("----------------Hi Burt!-----------------\r\n\r\n")

    try:
        while 1:
            ipt = input("cmd: ")
            if ipt == 'close':
                break
            elif ipt == 'home':
                burt.home()

            elif ipt == 't':
                demand_Joints = lm.grid_pos(burt,2,2,1,0)
                burt.movej(demand_Joints)        # move above brick
                burt.set_tcp(wp.lego_tcp_1brick)
                burt.movel_tool([0,0,0,0,-15*pi/180.0,0])       # rotate grabber
                burt.set_tcp(wp.lego_tcp)
                burt.home()

            # high level lego
            elif ipt == 'hi':
                burt.test()

            else:
                var = int(input("var: "))
                burt.serial_send(ipt,var,True)

    finally:
        print("Goodbye")
        if burt != 0:
            burt.close()
if __name__ == '__main__': main()


