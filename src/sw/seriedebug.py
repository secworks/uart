#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=======================================================================
#
# seriedebug.py
# -------------
# Program that sends test bytes onto the serial link and print out
# any response.
#
# Note: This proram requires the PySerial module.
# http://pyserial.sourceforge.net/
#
# 
# Author: Joachim Strömbergson
# Copyright (c) 2014  Secworks Sweden AB
# 
# Redistribution and use in source and binary forms, with or 
# without modification, are permitted provided that the following 
# conditions are met: 
# 
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer. 
# 
# 2. Redistributions in binary form must reproduce the above copyright 
#    notice, this list of conditions and the following disclaimer in 
#    the documentation and/or other materials provided with the 
#    distribution. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#=======================================================================

#-------------------------------------------------------------------
# Python module imports.
#-------------------------------------------------------------------
import sys
import serial

    
#-------------------------------------------------------------------
# main()
#
# Parse arguments.
#-------------------------------------------------------------------
def main():
    print "Seriedebug started..."
    print

    verbose = True
    
    # Create a serial devce and configure it.
    # Should be:
    # 1200 Baud
    # 1 start bit
    # 8 data bits
    # No parity bit
    # 2 stop bits
    # No RTS/CTS
    # Note: You need to update to the correct device in your system.
    ser = serial.Serial()
    ser.port='/dev/cu.usbserial-A801SA6T'
    ser.baudrate=9600
    ser.bytesize=8
    ser.parity='N'
    ser.stopbits=2
    ser.timeout=1
    ser.writeTimeout=0

    # Open the interface.
    ser.open()
    if verbose:
        print "Opening device."
        
    # Send a byte and try to get the response.
    test_byte = '\xaa'
    ser.write(test_byte)
    if verbose:
        print "transmitting byte 0x%0x" % ord(test_byte)

    
    if verbose:
        print "Waiting for response..."

    response = ""
    while not len(response):
        response = ser.read()
        if len(response):
            print "received response: 0x%0x" % ord(response)

    # Exit nicely.
    if verbose:
        print "Done. Closing device."
    ser.close()


#-------------------------------------------------------------------
# __name__
# Python thingy which allows the file to be run standalone as
# well as parsed from within a Python interpreter.
#-------------------------------------------------------------------
if __name__=="__main__": 
    # Run the main function.
    sys.exit(main())

#=======================================================================
# EOF seriedebug.py
#=======================================================================
