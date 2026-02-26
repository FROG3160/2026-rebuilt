# FROGlib/subsystem.py
from typing import Any, Callable, Optional, Type, Dict
import ntcore
from commands2 import Subsystem
from wpilib import DataLogManager
from wpiutil.log import DoubleLogEntry, BooleanLogEntry, StringLogEntry


class Tunable:
    """Descriptor for tunable NT values (editable from dashboard)"""

    def __init__(
        self,
        default: float | bool | str,
        topic_name: Optional[str] = None,
    ):
        self.default = default
        self.topic_name = topic_name
        self._attr_name = ""

    def __set_name__(self, owner, name):
        self._attr_name = name
        if self.topic_name is None:
            self.topic_name = name

    def _get_storage(self, obj: "FROGSubsystem") -> Dict[str, Any]:
        if not hasattr(obj, "_tunable_data"):
            setattr(obj, "_tunable_data", {})
        tunable_data = getattr(obj, "_tunable_data")
        if self._attr_name not in tunable_data:
            topic = self._get_typed_topic(
                obj.nt_tunables, self.topic_name, type(self.default)
            )
            pub = topic.publish()
            pub.set(self.default)
            sub = topic.subscribe(self.default)
            tunable_data[self._attr_name] = {
                "value": self.default,
                "pub": pub,
                "sub": sub,
            }
        return tunable_data[self._attr_name]

    def __get__(self, obj: "FROGSubsystem", objtype=None):
        if obj is None:
            return self
        storage = self._get_storage(obj)
        # Update local value from NetworkTables if it changed
        new_val = storage["sub"].get()
        if new_val != storage["value"]:
            storage["value"] = new_val
            # If the instance has a setter method (defined via @tunable_prop), call it
            setter_name = f"_set_{self._attr_name}"
            if hasattr(obj, setter_name):
                getattr(obj, setter_name)(new_val)
        return storage["value"]

    def __set__(self, obj: "FROGSubsystem", value):
        storage = self._get_storage(obj)
        storage["value"] = value
        storage["pub"].set(value)
        setter_name = f"_set_{self._attr_name}"
        if hasattr(obj, setter_name):
            getattr(obj, setter_name)(value)

    def _get_typed_topic(self, table: ntcore.NetworkTable, key: str, value_type: type):
        if value_type in (int, float):
            return table.getDoubleTopic(key)
        elif value_type is bool:
            return table.getBooleanTopic(key)
        else:
            return table.getStringTopic(key)


class Telemetry:
    """Descriptor for read-only telemetry (robot -> NT + optional logging)"""

    def __init__(
        self,
        getter: Callable[[Any], Any],
        topic_name: Optional[str] = None,
        log: bool = True,
    ):
        self.getter = getter
        self.topic_name = topic_name
        self.log = log
        self._attr_name = ""

    def __set_name__(self, owner, name):
        self._attr_name = name
        if self.topic_name is None:
            self.topic_name = name

    def _get_storage(self, obj: "FROGSubsystem") -> Dict[str, Any]:
        if not hasattr(obj, "_telemetry_data"):
            setattr(obj, "_telemetry_data", {})
        telemetry_data = getattr(obj, "_telemetry_data")
        if self._attr_name not in telemetry_data:
            # We need a sample value to determine type
            value = self.getter(obj)
            topic = self._get_typed_topic(obj.nt_table, self.topic_name, type(value))
            pub = topic.publish()
            log_entry = None
            if self.log:
                path = f"subsystems/{obj.getName()}/telemetry/{self.topic_name}"
                if isinstance(value, (int, float)):
                    log_entry = DoubleLogEntry(DataLogManager.getLog(), path)
                elif isinstance(value, bool):
                    log_entry = BooleanLogEntry(DataLogManager.getLog(), path)
                else:
                    log_entry = StringLogEntry(DataLogManager.getLog(), path)

            telemetry_data[self._attr_name] = {"pub": pub, "log": log_entry}
        return telemetry_data[self._attr_name]

    def __get__(self, obj: "FROGSubsystem", objtype=None):
        if obj is None:
            return self
        value = self.getter(obj)
        storage = self._get_storage(obj)
        storage["pub"].set(value)
        if storage["log"]:
            storage["log"].append(value)
        return value

    def _get_typed_topic(self, table: ntcore.NetworkTable, key: str, value_type: type):
        if value_type in (int, float):
            return table.getDoubleTopic(key)
        elif value_type is bool:
            return table.getBooleanTopic(key)
        else:
            return table.getStringTopic(key)


def tunable(default: float | bool | str, topic_name: Optional[str] = None):
    """
    Decorator for a method to act as a setter for a tunable value.
    Usage:
        @tunable(0.5)
        def my_value(self, val):
            self.hardware.set(val)
    """

    def decorator(func: Callable[[Any, Any], None]):
        return Tunable(default, topic_name)

    return decorator


def telemetry(topic_name: Optional[str] = None, log: bool = True):
    """
    Decorator for a method to act as a getter for telemetry.
    Usage:
        @telemetry()
        def motor_velocity(self):
            return self.motor.get_velocity()
    """

    def decorator(func: Callable[[Any], Any]):
        return Telemetry(func, topic_name, log)

    return decorator


class FROGSubsystem(Subsystem):
    """
    Base class for FROG Team subsystems with automatic NT tunables & telemetry.
    Inherit from this instead of plain Subsystem.
    """

    # Expose as class attributes for decorator access
    tunable: Callable[..., Callable[..., Tunable]] = tunable
    telemetry: Callable[..., Callable[..., Telemetry]] = telemetry

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Ensure name is set if not already set by Subsystem
        if not self.getName():
            self.setName(self.__class__.__name__)

        nt = ntcore.NetworkTableInstance.getDefault()
        self.nt_root = nt.getTable("subsystems")
        self.nt_table = self.nt_root.getSubTable(self.getName())
        self.nt_tunables = self.nt_table.getSubTable("tunables")
        self.nt_telemetry = self.nt_table.getSubTable("telemetry")

        # Discover all Telemetry and Tunable attributes once
        self._telemetry_attrs = []
        self._tunable_attrs = []
        for name in dir(self.__class__):
            attr = getattr(self.__class__, name)
            if isinstance(attr, Telemetry):
                self._telemetry_attrs.append(name)
            elif isinstance(attr, Tunable):
                self._tunable_attrs.append(name)

    @telemetry("Current Command", log=True)
    def current_command_telem(self) -> str:
        cmd = self.getCurrentCommand()
        return cmd.getName() if cmd else "None"

    @telemetry(".hasCommand", log=False)
    def has_command_telem(self) -> bool:
        return self.getCurrentCommand() is not None

    @telemetry("Default Command", log=True)
    def default_command_telem(self) -> str:
        cmd = self.getDefaultCommand()
        return cmd.getName() if cmd else "None"

    @telemetry(".hasDefault", log=False)
    def has_default_telem(self) -> bool:
        return self.getDefaultCommand() is not None

    def periodic(self) -> None:
        """Automatically updates all telemetry and checks for tunable updates."""
        for attr_name in self._telemetry_attrs:
            getattr(self, attr_name)  # Triggers Telemetry.__get__

        for attr_name in self._tunable_attrs:
            getattr(
                self, attr_name
            )  # Triggers Tunable.__get__ (which checks for NT updates)
