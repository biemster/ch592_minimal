#!/usr/bin/env python

import usb.core
import usb.util
import binascii

CH_USB_VENDOR_ID   = 0x1a86    # VID
CH_USB_PRODUCT_ID  = 0x8010    # PID
CH_USB_EP_OUT      = 0x01      # endpoint for command transfer out
CH_USB_EP_IN       = 0x81      # endpoint for reply transfer in
CH_USB_PACKET_SIZE = 64        # packet size
CH_USB_TIMEOUT     = 5000      # timeout for USB operations

CH_STR_PROG_DETECT = (0x81, 0x0d, 0x01, 0x01)

# Find the device
device = usb.core.find(idVendor=CH_USB_VENDOR_ID, idProduct=CH_USB_PRODUCT_ID)

if device is None:
    print("ch59x not found")
    exit(0)

# Get an endpoint instance
cfg = device.get_active_configuration()
intf = cfg[(0, 0)]

# Claim the interface
usb.util.claim_interface(device, intf)

device.write(CH_USB_EP_OUT, CH_STR_PROG_DETECT)
print( binascii.hexlify( device.read(CH_USB_EP_IN, CH_USB_PACKET_SIZE, CH_USB_TIMEOUT) ).decode())

# Release the interface when done
usb.util.release_interface(device, intf)
