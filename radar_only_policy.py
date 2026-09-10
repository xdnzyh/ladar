from __future__ import annotations


def install_radar_only_chassis_policy(app_cls) -> None:
    """Make hardware radar-only mode independent from the chassis serial port.

    The navigation UI keeps the configured chassis port for later automatic
    navigation, but radar-only connections must never open that port.  When the
    operator switches from navigation to radar-only, an already-open chassis
    port is released after any in-flight stop/settle sequence finishes.
    """
    if getattr(app_cls, "_radar_only_chassis_policy_installed", False):
        return

    original_build_hardware_controls = app_cls._build_hardware_controls
    original_set_view = app_cls._set_view
    original_connect = app_cls.connect

    def _set_chassis_port_visible(self, visible: bool) -> None:
        row = getattr(self, "_chassis_port_row", None)
        if row is None:
            combo = getattr(self, "chassis_combo", None)
            row = getattr(combo, "master", None)
        if row is None:
            return

        manager = row.winfo_manager()
        if visible:
            if manager:
                return
            options = {"fill": "x", "pady": 2}
            before = getattr(self, "_chassis_port_before", None)
            try:
                if before is not None and before.winfo_exists():
                    options["before"] = before
            except Exception:
                pass
            row.pack(**options)
        elif manager:
            row.pack_forget()

    def _release_chassis_for_radar(self) -> None:
        if getattr(self, "source", None) != "hardware":
            return
        view_mode = getattr(self, "view_mode", None)
        if view_mode is None or view_mode.get() != "radar":
            return

        endpoint = getattr(self, "chassis_endpoint", None)
        controller = getattr(self, "chassis_controller", None)
        if endpoint is None or not endpoint.is_open:
            status = getattr(self, "chassis_status_var", None)
            if status is not None:
                status.set("仅雷达模式：底盘串口未占用")
            return

        # Never cut the serial link while a STOP/DONE handshake is still in
        # flight.  The normal navigation code completes the safe stop first;
        # this policy releases the port immediately afterwards.
        if controller is not None and controller.in_flight:
            root = getattr(self, "root", None)
            if root is not None and not getattr(self, "_closed", False):
                root.after(50, self._release_chassis_for_radar)
            return

        if controller is not None:
            controller.disconnect()
        endpoint.close()
        status = getattr(self, "chassis_status_var", None)
        if status is not None:
            status.set("仅雷达模式：底盘串口已释放")
        logger = getattr(self, "_log", None)
        if callable(logger):
            logger("已切换到仅雷达；底盘串口已释放")

    def _build_hardware_controls(self, parent) -> None:
        original_build_hardware_controls(self, parent)
        combo = getattr(self, "chassis_combo", None)
        row = getattr(combo, "master", None)
        self._chassis_port_row = row
        self._chassis_port_before = None
        if row is not None:
            try:
                siblings = list(row.master.pack_slaves())
                index = siblings.index(row)
                if index + 1 < len(siblings):
                    self._chassis_port_before = siblings[index + 1]
            except (ValueError, AttributeError):
                pass
        _set_chassis_port_visible(self, getattr(self, "view_mode", None).get() == "navigation")

    def _set_view(self, view: str) -> None:
        original_set_view(self, view)
        if getattr(self, "source", None) != "hardware":
            return
        navigation = self.view_mode.get() == "navigation"
        _set_chassis_port_visible(self, navigation)
        if not navigation:
            _release_chassis_for_radar(self)

    def connect(self) -> None:
        if (getattr(self, "source", None) != "hardware"
                or getattr(self, "view_mode", None) is None
                or self.view_mode.get() != "radar"):
            return original_connect(self)

        chassis_var = getattr(self, "chassis_port_var", None)
        if chassis_var is None:
            return original_connect(self)

        # The saved configuration may still contain a valid chassis COM port.
        # Temporarily blank it so NavigationApp.connect() opens only measurement
        # and rotation endpoints in radar-only mode, then restore the saved value
        # for a later switch to automatic navigation.
        configured_chassis_port = chassis_var.get()
        chassis_var.set("")
        try:
            return original_connect(self)
        finally:
            chassis_var.set(configured_chassis_port)

    app_cls._set_chassis_port_visible = _set_chassis_port_visible
    app_cls._release_chassis_for_radar = _release_chassis_for_radar
    app_cls._build_hardware_controls = _build_hardware_controls
    app_cls._set_view = _set_view
    app_cls.connect = connect
    app_cls._radar_only_chassis_policy_installed = True
