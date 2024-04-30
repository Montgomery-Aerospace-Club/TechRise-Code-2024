# SPDX-FileCopyrightText: 2017 Limor Fried for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import time
import sdcardio
import board
import busio
import digitalio
import storage

# Connect to the card and mount the filesystem.
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
cs = board.D10
sdcard = sdcardio.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

# Use the filesystem as normal! Our files are under /sd

print("Writing to filesystem")
# append to the file!
while True:
    # open file for append
    with open("/sd/SDIOtest.txt", "a") as f:
        print("Writing")
        f.write("Hello World!\n")
    # file is saved
    time.sleep(2)
