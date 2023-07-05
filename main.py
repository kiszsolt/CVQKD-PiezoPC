#!/usr/bin/env python3
# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.



import sys

import numpy as np
# import pythonnet
import time
# import clr
from ctypes import *
import redpitaya_scpi as scpi
import matplotlib.pyplot as plt
# import matplotlib as mpl
# rcp = mpl.rcParams

CP = 0b00000000
IAbsMax = 3450
Imax = 1500
Tw = 0.3

# lib = cdll.LoadLibrary("C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPAX_64.dll")

# a = pythonnet.set_runtime('mono')
# print(a)
# pythonnet.load().AddReference("System.Windows.Forms")

def SVec(theta,phi):
    return np.array([np.sin(theta)*np.cos(phi),np.sin(theta)*np.sin(phi),np.cos(theta)])

def Sdiff(S1, S2):
    return np.linalg.norm(S1-S2)

def pos2b_vec(pos, b_vec):
    for i in [0,1,2]:
        b_vec[2*i] = CP | pos[i] >> 8
        b_vec[2*i+1] = 0xFF & pos[i]


def pos_check(pos):
    for i in [0,1,2]:
        if pos[i] <= 0:
            pos[i] = Imax-20

        if pos[i] >= Imax:
            pos[i] = 20

def DAC_w(b_vec, rp_s):
    send = str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
           + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ',' \
           + str(b_vec[6]) + ',' + str(b_vec[7])
    rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    time.sleep(Tw)


def Vread(rp_s):
    ## Scope start
    rp_s.tx_txt('ACQ:RST')
    rp_s.tx_txt('ACQ:DATA:UNITS VOLTS')
    rp_s.tx_txt('ACQ:DEC 4')
    rp_s.tx_txt('ACQ:SOUR1:GAIN HV')
    rp_s.tx_txt('ACQ:START')
    rp_s.tx_txt('ACQ:TRIG NOW')


    while 1:
        rp_s.tx_txt('ACQ:TRIG:STAT?')
        if rp_s.rx_txt() == 'TD':
            break

    ## UNIFIED OS
    # while 1:
    #     rp_s.tx_txt('ACQ:TRIG:FILL?')
    #     if rp_s.rx_txt() == '1':
    #         break

    rp_s.tx_txt('ACQ:SOUR1:DATA?')
    buff_string = rp_s.rx_txt()
    buff_string = buff_string.strip('{}\n\r').replace("  ", "").split(',')
    # return a list
    # return np.array(list(map(float, buff_string)))
    # return the average
    # plt.plot(list(map(float, buff_string)))
    # plt.show()
    return np.mean(list(map(float, buff_string))[6000:])

def Vread2(rp_s):
    ## Scope start
    rp_s.tx_txt('ACQ:RST')
    rp_s.tx_txt('ACQ:DATA:UNITS VOLTS')
    rp_s.tx_txt('ACQ:DEC 1')
    rp_s.tx_txt('ACQ:TRIG CH2_PE')
    rp_s.tx_txt('ACQ:TRIG:LEV 0.03')
    rp_s.tx_txt('ACQ:START')

    while 1:
        rp_s.tx_txt('ACQ:TRIG:STAT?')
        if rp_s.rx_txt() == 'TD':
            break

    ## UNIFIED OS
    # while 1:
    #     rp_s.tx_txt('ACQ:TRIG:FILL?')
    #     if rp_s.rx_txt() == '1':
    #         break

    rp_s.tx_txt('ACQ:SOUR1:DATA?')
    buff_string1 = rp_s.rx_txt()
    buff_string1 = buff_string1.strip('{}\n\r').replace("  ", "").split(',')

    rp_s.tx_txt('ACQ:SOUR2:DATA?')
    buff_string2 = rp_s.rx_txt()
    buff_string2 = buff_string2.strip('{}\n\r').replace("  ", "").split(',')

    return list(map(float, buff_string1)), list(map(float, buff_string2))

def deriv(pos, b_vec, rp_s, dp):
    n_pc = 0
    df = np.array([0.0,0.0,0.0])
    for p in pos:
        i2 = HighLim(p+dp)
        b_vec[2 * n_pc] = CP | i2 >> 8
        b_vec[2 * n_pc + 1] = 0xFF & i2
        DAC_w(b_vec, rp_s)
        y2 = Vread(rp_s)

        i1 = LowLim(p - dp)
        b_vec[2 * n_pc] = CP | i1 >> 8
        b_vec[2 * n_pc + 1] = 0xFF & i1
        DAC_w(b_vec, rp_s)
        y1 = Vread(rp_s)

        df[n_pc] = (y2 - y1) / (i2 - i1)

        i1 = p
        b_vec[2 * n_pc] = CP | i1 >> 8
        b_vec[2 * n_pc + 1] = 0xFF & i1

        n_pc = n_pc+1

    return df

def maximize(St, rng, steps, n_pc, b_vec, rp_s, instrumentHandle):
    scanID = c_int()
    S1 = c_double()
    S2 = c_double()
    S3 = c_double()
    s_vec = np.zeros((steps,3))

    k = 0
    resmax = 0.0
    dmin = 2.1
    for i in rng:
        b_vec[2*n_pc] = CP | i >> 8
        b_vec[2*n_pc+1] = 0xFF & i
        send =   str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
               + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ','\
               + str(b_vec[6]) + ',' + str(b_vec[7])
        rp_s.tx_txt('I2C:IO:W:B8 ' + send)
        time.sleep(Tw)


        buff = Vread(rp_s)
        bsize, = buff[7000:].shape
        res = sum(buff[7000:])/bsize
        if res > resmax:
            resmax = res
            i_min = i
            k_min = k


        # plt.plot(buff)
        # plt.ylabel('Voltage')
        # plt.show()


        ## scope finish

        # lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
        # lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
        # lib.TLPAX_releaseScan(instrumentHandle, scanID)

        #   print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
        # s_vec[k, :] = [S1.value, S2.value, S3.value]

        #  diff = Sdiff(s_vec[k, :], St)
        #  if diff < dmin:
        #     dmin = diff
        #     i_min = i
        #     k_min = k
        #
        # print(f'Sdiff[{k}] = {diff}')
        k = k + 1

    b_vec[2*n_pc] = CP | i_min >> 8
    b_vec[2*n_pc + 1] = 0xFF & i_min
    send = str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
           + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ',' \
           + str(b_vec[6]) + ',' + str(b_vec[7])
    rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    time.sleep(Tw)

    # lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    # lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    # lib.TLPAX_releaseScan(instrumentHandle, scanID)

    buff = Vread(rp_s)
    bsize, = buff[7000:].shape
    res = sum(buff[7000:]) / bsize
    var = np.sqrt(buff[7000:].var())
    print(f'Vmax @ [k_min = {k_min}] = {res}, VAR = {var}')
    return i_min

def minimize(St, rng, steps, n_pc, b_vec, rp_s, instrumentHandle):
    k = 0
    resmin = 10000.0
    for i in rng:
        b_vec[2*n_pc] = CP | i >> 8
        b_vec[2*n_pc+1] = 0xFF & i
        send =   str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
               + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ','\
               + str(b_vec[6]) + ',' + str(b_vec[7])
        rp_s.tx_txt('I2C:IO:W:B8 ' + send)
        time.sleep(Tw)

        buff = Vread(rp_s)
        res = np.sqrt(np.var(buff[7100:]))

        plt.plot(buff)
        plt.ylabel('Voltage')
        plt.show()

        if res < resmin:
            resmin = res
            i_min = i
            k_min = k

        k = k + 1

    b_vec[2*n_pc] = CP | i_min >> 8
    b_vec[2*n_pc + 1] = 0xFF & i_min
    send = str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
           + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ',' \
           + str(b_vec[6]) + ',' + str(b_vec[7])
    rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    time.sleep(Tw)

    buff = Vread(rp_s)
    res =  np.sqrt(np.var(buff[7100:]))
    print(f'Vmax @ [k_min = {k_min}] = {res}')
    return i_min

def minimize2(St, rng, steps, n_pc, b_vec, rp_s, instrumentHandle):
    k = 0
    resmin = 10000.0
    for i in rng:
        b_vec[2*n_pc] = CP | i >> 8
        b_vec[2*n_pc+1] = 0xFF & i
        send =   str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
               + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ','\
               + str(b_vec[6]) + ',' + str(b_vec[7])
        rp_s.tx_txt('I2C:IO:W:B8 ' + send)
        time.sleep(Tw)

        buff1, buff2 = Vread2(rp_s)



        plt.plot(buff1)
        plt.plot(buff2)
        plt.ylabel('Voltage')
        plt.show()

        res = np.sqrt(np.var(buff1[7100:]))

        if res < resmin:
            resmin = res
            i_min = i
            k_min = k

        k = k + 1

    b_vec[2*n_pc] = CP | i_min >> 8
    b_vec[2*n_pc + 1] = 0xFF & i_min
    send = str(b_vec[0]) + ',' + str(b_vec[1]) + ',' + str(b_vec[2]) + ',' \
           + str(b_vec[3]) + ',' + str(b_vec[4]) + ',' + str(b_vec[5]) + ',' \
           + str(b_vec[6]) + ',' + str(b_vec[7])
    rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    time.sleep(Tw)

    buff = Vread(rp_s)
    res =  np.sqrt(np.var(buff[7100:]))
    print(f'Vmax @ [k_min = {k_min}] = {res}')
    return i_min

def LowLim(i):
    return max(i,0)

def HighLim(i):
    return min(i,Imax)

def main():
    rp_s = scpi.scpi(sys.argv[1])

    if (len(sys.argv) > 2):
        led = int(sys.argv[2])
    else:
        led = 0

    # print ("Blinking LED["+str(led)+"]")

    # period = 1 # seconds

    # rp_s.tx_txt('ANALOG:PIN AOUT0,0.5')
    #
    # while 1:
    #     time.sleep(period/2.0)
    #     rp_s.tx_txt('DIG:PIN LED' + str(led) + ',' + str(1))
    #     time.sleep(period/2.0)
    #     rp_s.tx_txt('DIG:PIN LED' + str(led) + ',' + str(0))

    AddrW = 0b11000000
    AddrR = 0b11000001

    rp_s.tx_txt('I2C:DEV' + str(AddrW >> 1) + ' "/dev/i2c-0"')
    rp_s.tx_txt('I2C:DEV?')
    print(rp_s.rx_txt())
    rp_s.tx_txt('I2C:FMODE ON')

    CWD = 0b01000000
    VPG = 0b10010000

    # fat write
    dataA = int(100)  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataB = int(100)  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataC = int(100)  # 3450 is the max value;  Vref bit = 1, gain bit = 0, after the 3x amplifier it yields 5V
    dataD = 0

    pos = np.array([dataA, dataB, dataC])

    # CP = 0b00000000
    b1 = CP | dataA >> 8
    b2 = 0xFF & dataA
    b3 = CP | dataB >> 8
    b4 = 0xFF & dataB
    b5 = CP | dataC >> 8
    b6 = 0xFF & dataC
    b7 = CP | dataD >> 8
    b8 = 0xFF & dataD

    Vref = 0b10000000 | 0b1111  # 1 =>
    G = 0b11000000 | 0b0000
    rp_s.tx_txt('I2C:IO:W:B1 ' + str(G))
    rp_s.tx_txt('I2C:IO:W:B1 ' + str(Vref))

    send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
        b7) + ',' + str(b8)
    rp_s.tx_txt('I2C:IO:W:B8 ' + send)



    """--------------------------------Polariméter-----------------------------"""
    # Load DLL library
    # lib = cdll.LoadLibrary("C:\Program Files\IVI Foundation\VISA\Win64\Bin\TLPAX_64.dll")

    # Detect and initialize PAX1000 device
    instrumentHandle = c_ulong()
    # IDQuery = True
    # resetDevice = False
    # resource = c_char_p(b"")
    # deviceCount = c_int()
    #
    # # Check how many PAX1000 are connected
    # lib.TLPAX_findRsrc(instrumentHandle, byref(deviceCount))
    # if deviceCount.value < 1:
    #     print("No PAX1000 device found.")
    #     exit()
    # else:
    #     print(deviceCount.value, "PAX1000 device(s) found.")
    #     print("")
    #
    # # Connect to the first available PAX1000
    # lib.TLPAX_getRsrcName(instrumentHandle, 0, resource)
    # if (0 == lib.TLPAX_init(resource.value, IDQuery, resetDevice, byref(instrumentHandle))):
    #     print("Connection to first PAX1000 initialized.")
    # else:
    #     print("Error with initialization.")
    #     exit()
    # print("")
    #
    # # Short break to make sure the device is correctly initialized
    # time.sleep(5)
    #
    # # Make settings
    # lib.TLPAX_setMeasurementMode(instrumentHandle, 9)
    # lib.TLPAX_setWavelength(instrumentHandle, c_double(1550e-9))  # hullámhossz beállítás
    # lib.TLPAX_setBasicScanRate(instrumentHandle, c_double(60))
    # lib.TLPAX_setPowerRange(instrumentHandle, c_double(0.01));
    #
    # # Check settings
    # wavelength = c_double()
    # lib.TLPAX_getWavelength(instrumentHandle, byref(wavelength))
    # print("Set wavelength [nm]: ", wavelength.value * 1e9)
    # mode = c_int()
    # lib.TLPAX_getMeasurementMode(instrumentHandle, byref(mode))
    # print("Set mode: ", mode.value)
    # scanrate = c_double()
    # lib.TLPAX_getBasicScanRate(instrumentHandle, byref(scanrate))
    # print("Set scanrate: ", scanrate.value)
    # print("")

    steps =61
    rng = np.linspace(0, Imax, steps, dtype='int')
    rng_vec = np.array([rng, rng, rng])
    di = round(Imax/(steps-1))
    di_vec = np.array([di, di, di])
    # St = SVec(0.5*np.pi, 0.3*np.pi)
    St = np.array([0, -1.0, 0])
    b_vec = np.array([b1, b2, b3, b4, b5, b6, b7, b8])
    i_min = np.empty([3])

    ##
    ## Derivation method
    ##
    dfscale = 20000
    avg=0
    dp=10
    for j in range(0,100):
        df = deriv(pos, b_vec, rp_s, dp)*dfscale
        dfnorm = np.linalg.norm(df)
        if dfnorm > 30:
            df = (df*30.0/dfnorm).astype(int)

        if dfnorm < 2:
            df = (df*2.0/dfnorm).astype(int)

        # avg = np.linalg.norm(df)*0.3+avg*0.7
        # if j>30 and avg<20:
        #     dfscale = 3000
        #     dp = 5

        # if j>30 and avg<3:
        #     dfscale = 1000
        #     dp = 2


        # if j>30 and avg<1:
        #     dfscale = 15000

        pos = (pos+df).astype(int)
        if np.min(pos)<0 or np.max(pos)>Imax:
            pos_check(pos)

        print(j, pos, Vread(rp_s), np.linalg.norm(df), avg)
        pos2b_vec(pos, b_vec)
        DAC_w(b_vec, rp_s)

    exit()


    for i in [0,1,2]:
        for n_pc in [0,1,2]:
            i_min[n_pc] = maximize(St, rng_vec[n_pc], steps, n_pc, b_vec, rp_s, instrumentHandle)

        steps = 31
        rng_vec = np.empty([3,steps],dtype='int')
        for j in [0,1,2]:
            ll = int(LowLim(i_min[j]-4*di_vec[j]))
            hl = int(HighLim(i_min[j]+4*di_vec[j]))
            rng_vec[j] = np.round(np.linspace(ll, hl, steps))
            di_vec[j] = round((hl-ll)/(steps-1))



    scanID = c_int()

    S1 = c_double()  ### fontos sor
    S2 = c_double()  ### fontos sor
    S3 = c_double()  ### fontos sor
    #
    # steps = 41
    # ii = np.linspace(0,Imax, steps, dtype='int')
    # time.sleep(0.25)
    # s_vec = np.zeros((steps,3))
    # St = SVec(0.5, 1.2)
    # # Tw = 0.3
    #
    # print("First squeezer")
    # dmin = 2.1
    # k=0
    # for i in ii:
    #     b1 = CP | i >> 8
    #     b2 = 0xFF & i
    #     send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #         b7) + ',' + str(b8)
    #     rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    #     time.sleep(Tw)
    #
    #     lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    #     lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    #     lib.TLPAX_releaseScan(instrumentHandle, scanID)
    #
    # #    print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
    #     s_vec[k,:] = [S1.value, S2.value, S3.value]
    #
    #     diff = Sdiff(s_vec[k,:],St)
    #     if diff<dmin:
    #         dmin = diff
    #         i_min = i
    #
    #     print(f'Sdiff = {diff}')
    #     k = k+1
    #
    #
    # b1 = CP | i_min >> 8
    # b2 = 0xFF & i_min
    # send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #        b7) + ',' + str(b8)
    # rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    # time.sleep(Tw)
    #
    # print('Second squeezer')
    # dmin = 2.1
    # k = 0
    # for i in ii:
    #     b3 = CP | i >> 8
    #     b4 = 0xFF & i
    #     send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #         b7) + ',' + str(b8)
    #     rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    #     time.sleep(Tw)
    #
    #     lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    #     lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    #     lib.TLPAX_releaseScan(instrumentHandle, scanID)
    #
    #     #   print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
    #     s_vec[k, :] = [S1.value, S2.value, S3.value]
    #
    #     diff = Sdiff(s_vec[k, :], St)
    #     if diff < dmin:
    #         dmin = diff
    #         i_min = i
    #
    #     print(f'Sdiff = {diff}')
    #     k = k + 1
    #
    # b3 = CP | i_min >> 8
    # b4 = 0xFF & i_min
    # send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #     b7) + ',' + str(b8)
    # rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    # time.sleep(Tw)
    #
    # print('Third squeezer')
    # dmin = 2.1
    # k = 0
    # for i in ii:
    #     b5 = CP | i >> 8
    #     b6 = 0xFF & i
    #     send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #         b7) + ',' + str(b8)
    #     rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    #     time.sleep(Tw)
    #
    #     lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    #     lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    #     lib.TLPAX_releaseScan(instrumentHandle, scanID)
    #
    #     #    print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")
    #     s_vec[k, :] = [S1.value, S2.value, S3.value]
    #
    #     diff = Sdiff(s_vec[k, :], St)
    #     if diff < dmin:
    #         dmin = diff
    #         i_min = i
    #
    #     print(f'Sdiff = {diff}')
    #     k = k + 1
    #
    # b5 = CP | i_min >> 8
    # b6 = 0xFF & i_min
    # send = str(b1) + ',' + str(b2) + ',' + str(b3) + ',' + str(b4) + ',' + str(b5) + ',' + str(b6) + ',' + str(
    #     b7) + ',' + str(b8)
    # rp_s.tx_txt('I2C:IO:W:B8 ' + send)
    # time.sleep(Tw)

    # lib.TLPAX_getLatestScan(instrumentHandle, byref(scanID))
    # lib.TLPAX_getStokesNormalized(instrumentHandle, scanID.value, byref(S1), byref(S2), byref(S3))
    # lib.TLPAX_releaseScan(instrumentHandle, scanID)

    s = np.array([S1.value, S2.value, S3.value])
    diff = Sdiff(s, St)
    print(f'Sdiff = {diff}')

 #   plt.plot(ii, SVec[:, 0])
 #   plt.plot(ii, s_vec[:,0], ii, s_vec[:,1], ii, s_vec[:,2])
 #   plt.show()

    print(f"Svec=  {S1.value}, {S2.value}, {S3.value}\n")

    """-----------Polariméter---------"""
    # Close
    # lib.TLPAX_close(instrumentHandle)
    # print("Connection to PAX1000 closed.")
    # """-------------------------------"""

    rp_s.close()

if __name__ == "__main__":
    main()
