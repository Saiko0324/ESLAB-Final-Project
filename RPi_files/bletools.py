import dbus

BLUEZ_SERVICE_NAME = "org.bluez"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"

class BleTools:
    @classmethod
    def get_bus(cls):
        return dbus.SystemBus()

    @classmethod
    def find_adapter(cls, bus):
        remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE)
        objects = remote_om.GetManagedObjects()

        for path, interfaces in objects.items():
            if LE_ADVERTISING_MANAGER_IFACE in interfaces:
                return path

        return None
