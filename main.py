from machine import Pin, I2C, ADC
from time import sleep
import sys

# change these pin definitions to suit your wiring! this is for the Altmode Friend module
i2c = I2C(sda=Pin(18), scl=Pin(19), id=1, freq=400000)
print(i2c.scan()) # debugging whether FUSB has been found

# A few helper functions

def read_cc(cc):
    # enable a CC pin for reading
    assert(cc in [1, 2])
    x = i2c.readfrom_mem(0x22, 0x02, 1)[0]
    x1 = x
    clear_mask = ~0b1100 & 0xFF
    x &= clear_mask
    mask = 0b1000 if cc == 2 else 0b100
    x |= mask
    i2c.writeto_mem(0x22, 0x02, bytes((x,)) )

def rxb_state():
    # get read buffer interrupt states - (rx buffer empty, rx buffer full)
    st = i2c.readfrom_mem(0x22, 0x41, 1)[0]
    return ((st & 0b100000) >> 5, (st & 0b10000) >> 4)

def get_rxb(l=80):
    # read the FIFO contents
    return i2c.readfrom_mem(0x22, 0x43, l)

# Startup sequence

# reset the entire FUSB
i2c.writeto_mem(0x22, 0xc, bytes([0b1]))
# enables all power circuits
i2c.writeto_mem(0x22, 0x0b, bytes([0b1111]))
# unmasks all interrupts
i2c.writeto_mem(0x22, 0xa, bytes([0b0]))
i2c.writeto_mem(0x22, 0xe, bytes([0b0]))
i2c.writeto_mem(0x22, 0xf, bytes([0b0]))

# configuration
ctrl0 = 0b00000000 # unmask all interrupts; don't autostart TX.. disable pullup current
i2c.writeto_mem(0x22, 0x06, bytes((ctrl0,)) )
ctrl3 = 0b00000111 # enable automatic packet retries
i2c.writeto_mem(0x22, 0x09, bytes((ctrl3,)) )

# read CC pins and see which one senses the pullup
read_cc(1)
sleep(0.001)
cc1_c = i2c.readfrom_mem(0x22, 0x40, 1)[0] & 0b11
read_cc(2)
sleep(0.001)
cc2_c = i2c.readfrom_mem(0x22, 0x40, 1)[0] & 0b11
cc = [1, 2][cc1_c < cc2_c]
# flush receive
ctrl1 = 0b00000100
i2c.writeto_mem(0x22, 0x07, bytes((ctrl1,)) )
# enable transmit on either CC1 or CC2, whichever is being used
x = i2c.readfrom_mem(0x22, 0x03, 1)[0]
x1 = x
mask = 0b10 if cc == 2 else 0b1
x &= 0b11111100 # clearing both TX bits
x |= mask
x |= 0b100
i2c.writeto_mem(0x22, 0x03, bytes((x,)) )
# flush transmit
ctrl0 = 0b01000100
i2c.writeto_mem(0x22, 0x06, bytes((ctrl0,)) )
# flush receive again
ctrl1 = 0b00000100
i2c.writeto_mem(0x22, 0x07, bytes((ctrl1,)) )
# resets the FUSB PD logic
i2c.writeto_mem(0x22, 0xc, bytes([0b10]))

# now we're ready to receive!

pdo_requested = False
pdos = []

# this is a 'main loop' of sorts
def wait():
  global pdo_requested, pdos
  while True:
    if rxb_state()[0] == 0: # == buffer not empty; as '1' indicates empty buffer
      if not pdo_requested:
        pdos = read_pdos()
        # this is where we hook a function that selects a proper PDO
        pdo_i, current = select_pdo(pdos)
        request_pdo(pdo_i, current, current)
        print("PDO requested!")
        pdo_requested = True
  # this will run in a very tight loop. good for USB-PD,
  # but, having interrupts would let us be more efficient about it

pdo_types = ['fixed', 'batt', 'var', 'pps']
pps_types = ['spr', 'epr', 'res', 'res']

def read_pdos():
    pdo_list = []
    header = get_rxb(1)[0]
    print(header)
    assert(header == 0xe0)
    b1, b0 = get_rxb(2)
    pdo_count = (b0 >> 4) & 0b111
    read_len = pdo_count*4
    pdos = get_rxb(read_len)
    _ = get_rxb(4) # crc
    for pdo_i in range(pdo_count):
        pdo_bytes = pdos[(pdo_i*4):][:4]
        parsed_pdo = parse_pdo(pdo_bytes)
        pdo_list.append(parsed_pdo)
    return pdo_list

def parse_pdo(pdo):
    pdo_t = pdo_types[pdo[3] >> 6]
    if pdo_t == 'fixed':
        current_h = pdo[1] & 0b11
        current_b = ( current_h << 8 ) | pdo[0]
        current = current_b * 10
        voltage_h = pdo[2] & 0b1111
        voltage_b = ( voltage_h << 6 ) | (pdo[1] >> 2)
        voltage = voltage_b * 50
        peak_current = (pdo[2] >> 4) & 0b11
        return (pdo_t, voltage, current, peak_current, pdo[3])
    elif pdo_t in ['batt', 'var']:
        # TODO am not motivated to parse these
        return (pdo_t, pdo)
    elif pdo_t == 'pps':
        t = (pdo[3] >> 4) & 0b11
        limited = (pdo[3] >> 5) & 0b1
        max_voltage_h = pdo[3] & 0b1
        max_voltage_b = (max_voltage_h << 7) | pdo[2] >> 1
        max_voltage = max_voltage_b * 100
        min_voltage = pdo[1] * 100
        max_current_b = pdo[0] & 0b1111111
        max_current = max_current_b * 50
        return ('pps', pps_types[t], max_voltage, min_voltage, max_current, limited)

def request_pdo(num, current, max_current, msg_id=0):
    sop_seq = [0x12, 0x12, 0x12, 0x13, 0x80]
    eop_seq = [0xff, 0x14, 0xfe, 0xa1]
    obj_count = 1
    pdo_len = 2 + (4*obj_count)
    pdo = [0 for i in range(pdo_len)]

    pdo[0] |= 0b10 << 6 # PD 3.0
    pdo[0] |= 0b00010 # request

    pdo[1] |= obj_count << 4

    # packing max current into fields
    max_current_b = max_current // 10
    max_current_l = max_current_b & 0xff
    max_current_h = max_current_b >> 8
    pdo[2] = max_current_l
    pdo[3] |= max_current_h

    # packing current into fields
    current_b = current // 10
    current_l = current_b & 0x3f
    current_h = current_b >> 6
    pdo[3] |= current_l << 2
    pdo[4] |= current_h

    pdo[5] |= (num+1) << 4 # object position
    pdo[5] |= (msg_id) << 1 # message ID
    pdo[5] |= 0b1 # no suspend

    sop_seq[4] |= pdo_len

    i2c.writeto_mem(0x22, 0x43, bytes(sop_seq) )
    i2c.writeto_mem(0x22, 0x43, bytes(pdo) )
    i2c.writeto_mem(0x22, 0x43, bytes(eop_seq) )

# general helper functions

def myhex(b, j=" "):
    # print a 'bytes' object as hex
    l = []
    for e in b:
        e = hex(e)[2:].upper()
        if len(e) < 2:
            e = ("0"*(2-len(e)))+e
        l.append(e)
    return j.join(l)

def mybin(b, j=" "):
    # print a 'bytes' object as binary
    l = []
    for e in b:
        e = bin(e)[2:].upper()
        if len(e) < 8:
            e = ("0"*(8-len(e)))+e
        l.append(e)
    return j.join(l)

# example PDO select function - the simplest case, "find 9V"

def select_pdo(pdos):
    for i, pdo in enumerate(pdos):
        if pdo[0] != 'fixed': # skipping variable PDOs for now
            pass
        voltage = pdo[1]
        if voltage == 9000:
            return (i, 500) # we will not need a lot for test purposes
    # you can do "light an LED up if the 9V PDO is not found" here,
    # or, to be honest, you can do whatever. switch GPIOs,
    # send extra PD messages, do something else entirely -
    # it's your world now.

# a more complicated PDO select function, with actually useful behaviour!
"""
def select_pdo(pdos):
    # finding a PDO with maximum extractable power
    # for a given static resistance,
    # while making sure that we don't overcurrent the PSU
    resistance = 8
    # calculation storage lists
    power_levels = []
    currents = []
    for pdo in pdos:
        if pdo[0] != 'fixed': # skipping variable PDOs for now
            # keeping indices in sync
            power_levels.append(0); currents.append(0)
            continue
        t, voltage, max_current, oc, flags = pdo
        voltage = voltage / 1000
        max_current = max_current / 1000
        # calculating the power needed
        current = voltage / resistance
        current = current * 1.10 # adding 10% leeway
        if current > max_current: # current too high, skipping
            # keeping indices in sync
            power_levels.append(0); currents.append(0)
            continue
        power = voltage * current
        power_levels.append(power)
        currents.append(int(current*1000))
    # finding the maximum power level
    i = power_levels.index(max(power_levels))
    # returning the PDO index + current we'd need
    return i, currents[i]
"""

# now, let's launch our main loop
wait()
