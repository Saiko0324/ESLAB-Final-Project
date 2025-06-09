import dbus
import dbus.service
from bletools import BleTools

BLUEZ_SERVICE_NAME = "org.bluez"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"
LE_ADVERTISEMENT_IFACE = "org.bluez.LEAdvertisement1"

class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.freedesktop.DBus.Error.InvalidArgs"

class Advertisement(dbus.service.Object):
    PATH_BASE = "/org/bluez/example/advertisement"

    def __init__(self, index, advertising_type):
        self.path = self.PATH_BASE + str(index)
        self.bus = BleTools.get_bus()
        self.ad_type = advertising_type
        self.local_name = None
        self.service_uuids = []
        dbus.service.Object.__init__(self, self.bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        properties = {
            "Type": self.ad_type,
        }
        if self.local_name:
            properties["LocalName"] = dbus.String(self.local_name)
        if self.service_uuids:
            properties["ServiceUUIDs"] = dbus.Array(self.service_uuids, signature='s')
        return {LE_ADVERTISEMENT_IFACE: properties}

    def add_local_name(self, name):
        self.local_name = dbus.String(name)

    def add_service_uuid(self, uuid):
        self.service_uuids.append(uuid)

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature="s",
                         out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE,
                         in_signature='',
                         out_signature='')
    def Release(self):
        print(f'{self.path}: Released!')

    def register(self):
        adapter = BleTools.find_adapter(self.bus)
        ad_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE_NAME, adapter),
            LE_ADVERTISING_MANAGER_IFACE)
        ad_manager.RegisterAdvertisement(
            self.get_path(), {},
            reply_handler=lambda: print("Advertisement registered"),
            error_handler=lambda e: print(f"Failed to register advertisement: {e}")
        )
