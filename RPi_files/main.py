#!/usr/bin/python3

import dbus
from advertisement import Advertisement
from service import Application

class BeaconAdvertisement(Advertisement):
    def __init__(self, index):
        super().__init__(index, "peripheral")
        self.add_local_name("RPi_1")
        self.include_tx_power = True

app = Application()
adv = BeaconAdvertisement(0)

adv.register()

try:
    app.run()
except KeyboardInterrupt:
    app.quit()