import cv2
import numpy as np
from pyModbusTCP.client import ModbusClient
import time
SERVER_HOST = "192.254.133.178"
SERVER_PORT = 502

c = ModbusClient()
c.host(SERVER_HOST)
c.port(SERVER_PORT)
a=0
A=[0]

if not c.is_open():
    if not c.open():
        print("unable to connect to " + SERVER_HOST + ":" + str(SERVER_PORT))
if c.is_open():
    while(1):
        A = c.read_holding_registers(3)
        print("Connected")

        while a==0 and A==[111]:
            print("Congratulations")