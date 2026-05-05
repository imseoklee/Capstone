import json
import omni.physx as _physx
import omni.timeline
import omni.usd
import os
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

try:
    from pxr import PhysxSchema
except ImportError:  # pragma: no cover - only resolved inside Isaac Sim
    PhysxSchema = None

ROBOT_PRIM_BY_NAMESPACE = {
    "iw_hub_ros": "/iw_hub_ROS",
    "iw_hub_ros_01": "/iw_hub_ROS_01",
}

HIGH_FRICTION_VALUE = 10000.0
PHYSICS_MATERIAL_PRIM_PATH = "/Root/Looks/CapstoneHighFrictionPhysics"
PALLET_PRIM_PREFIX = "PalletBin_"
LIFT_CONTACT_PRIM_PATHS = [
    "/iw_hub_ROS/lift",
    "/iw_hub_ROS_01/lift",
]

LIFT_CMD_PATH = "/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/config/lift_cmd.json"
LIFT_STATE_PATH = "/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/config/lift_state.json"
TARGET_VISUAL_CMD_PATH = "/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/config/target_visual.json"
BRIDGE_LOG_PATH = "/home/nsl/Capston_workspace/isaac_lift_bridge_log"
LIFT_JOINT_NAME = "lift_joint"
POLL_INTERVAL = 1
LIFT_RAMP_DURATION_SEC = 0.5
LIFT_DRIVE_MAX_FORCE = 100000.0
LIFT_DRIVE_STIFFNESS = 220000.0
LIFT_DRIVE_DAMPING = 12000.0


def _emit_bridge_log(message: str) -> None:
    print(message)
    try:
        log_dir = os.path.dirname(BRIDGE_LOG_PATH)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        with open(BRIDGE_LOG_PATH, "a", encoding="utf-8") as handle:
            handle.write(f"{message}\n")
    except Exception:
        pass


class LiftBridgeRuntime:
    def __init__(self) -> None:
        self._dc = None
        self._articulations = {}
        self._joints = {}
        self._joint_indices = {}
        self._targets = {namespace: 0.0 for namespace in ROBOT_PRIM_BY_NAMESPACE}
        self._applied_targets = {namespace: 0.0 for namespace in ROBOT_PRIM_BY_NAMESPACE}
        self._ramp_start_positions = {namespace: 0.0 for namespace in ROBOT_PRIM_BY_NAMESPACE}
        self._ramp_start_times = {namespace: 0.0 for namespace in ROBOT_PRIM_BY_NAMESPACE}
        self._last_logged_targets = {namespace: None for namespace in ROBOT_PRIM_BY_NAMESPACE}
        self._highlighted_target_prim = None
        self._highlight_original_colors = {}
        self._highlight_original_material_targets = {}
        self._highlight_root_original_material_targets = None
        self._last_target_visual_cmd = None
        self._last_target_visual_mtime = self._get_target_visual_mtime()
        self._timeline_subscription = None
        self._step_count = 0
        self._sim_time = 0.0
        self._write_cmd({namespace: 0.0 for namespace in ROBOT_PRIM_BY_NAMESPACE})
        self._init_dynamic_control()
        self._write_state({namespace: 0.0 for namespace in ROBOT_PRIM_BY_NAMESPACE})
        self._subscribe_timeline_events()
        self._log_startup_status()
        self._apply_high_friction_material()
        self._clear_stale_target_highlights()
        self._log_target_visual_watch_status()
        self._physx_subscription = _physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )
        _emit_bridge_log("[INFO] Isaac Sim file lift bridge started.")

    def shutdown(self) -> None:
        self._restore_previous_target_highlight()
        self._timeline_subscription = None
        self._physx_subscription = None
        self._dc = None

    def _init_dynamic_control(self) -> None:
        try:
            from omni.isaac.dynamic_control import _dynamic_control

            self._dc = _dynamic_control.acquire_dynamic_control_interface()
        except ImportError:
            try:
                import omni.isaac.dynamic_control._dynamic_control as _dc

                self._dc = _dc.acquire_dynamic_control_interface()
            except Exception as exc:  # noqa: BLE001
                _emit_bridge_log(f"[ERROR] Dynamic Control import failed: {exc}")
                return

        for namespace, prim_path in ROBOT_PRIM_BY_NAMESPACE.items():
            art = self._dc.get_articulation(prim_path)
            if art == 0:
                _emit_bridge_log(f"[ERROR] Articulation not found: {prim_path}")
                stage = omni.usd.get_context().get_stage()
                prim = stage.GetPrimAtPath(prim_path)
                _emit_bridge_log(f"  prim valid={prim.IsValid() if prim else False}")
                continue

            self._articulations[namespace] = art
            self._dc.wake_up_articulation(art)
            dof_count = self._dc.get_articulation_dof_count(art)
            joint = self._dc.find_articulation_dof(art, LIFT_JOINT_NAME)

            if joint == 0:
                found = False
                for i in range(dof_count):
                    dof = self._dc.get_articulation_dof(art, i)
                    dof_name = self._dc.get_dof_name(dof)
                    if "lift" in dof_name.lower():
                        self._joints[namespace] = dof
                        self._joint_indices[namespace] = i
                        _emit_bridge_log(
                            f"[INFO] lift DOF found for {namespace}: {dof_name} (index {i})"
                        )
                        found = True
                        break
                if not found:
                    _emit_bridge_log(f"[ERROR] Could not find lift DOF on {prim_path}")
            else:
                self._joints[namespace] = joint
                for i in range(dof_count):
                    dof = self._dc.get_articulation_dof(art, i)
                    if dof == joint:
                        self._joint_indices[namespace] = i
                        break
                _emit_bridge_log(f"[INFO] Bound {namespace} lift joint on {prim_path}")

            self._configure_lift_drive(namespace)

    def _subscribe_timeline_events(self) -> None:
        try:
            timeline = omni.timeline.get_timeline_interface()
            event_stream = timeline.get_timeline_event_stream()
            self._timeline_subscription = event_stream.create_subscription_to_pop(
                self._on_timeline_event
            )
        except Exception as exc:  # noqa: BLE001
            _emit_bridge_log(f"[WARN] Failed to subscribe to Isaac Sim timeline events: {exc}")

    def _on_timeline_event(self, event) -> None:
        try:
            event_type = int(event.type)
        except Exception:
            return

        if event_type == int(omni.timeline.TimelineEventType.STOP):
            _emit_bridge_log("[INFO] Isaac Sim timeline stopped. Restoring target pallet highlight.")
            self._restore_previous_target_highlight()
            self._clear_stale_target_highlights()
        elif event_type == int(omni.timeline.TimelineEventType.PLAY):
            _emit_bridge_log("[INFO] Isaac Sim timeline started. Waiting for a fresh target pallet command.")
            self._clear_stale_target_highlights()
            self._last_target_visual_mtime = self._get_target_visual_mtime()
            self._last_target_visual_cmd = None

    def _log_startup_status(self) -> None:
        _emit_bridge_log("[INFO] Lift bridge connection status:")
        for namespace, prim_path in ROBOT_PRIM_BY_NAMESPACE.items():
            art = self._articulations.get(namespace)
            joint = self._joints.get(namespace)
            if art and joint:
                _emit_bridge_log(
                    f"[INFO]   {namespace}: connected to {prim_path} and lift_joint is ready."
                )
            elif art:
                _emit_bridge_log(
                    f"[WARN]   {namespace}: articulation found on {prim_path}, but lift_joint is not ready."
                )
            else:
                _emit_bridge_log(
                    f"[WARN]   {namespace}: articulation connection failed for {prim_path}."
                )
        _emit_bridge_log(
            f"[INFO] Lift motion config: ramp_duration={LIFT_RAMP_DURATION_SEC:.2f}s, poll_interval={POLL_INTERVAL} step(s)"
        )
        _emit_bridge_log(
            "[INFO] Lift drive config: max_force=%.1f stiffness=%.1f damping=%.1f"
            % (
                LIFT_DRIVE_MAX_FORCE,
                LIFT_DRIVE_STIFFNESS,
                LIFT_DRIVE_DAMPING,
            )
        )
        _emit_bridge_log(
            f"[INFO] Requested high-friction physics material: static/dynamic friction={HIGH_FRICTION_VALUE:.1f}"
        )
        _emit_bridge_log(f"[INFO] Target visual bridge is watching {TARGET_VISUAL_CMD_PATH}")

    def _configure_lift_drive(self, namespace: str) -> None:
        art = self._articulations.get(namespace)
        dof_index = self._joint_indices.get(namespace)
        if self._dc is None or art is None or dof_index is None:
            return

        get_props = getattr(self._dc, "get_articulation_dof_properties", None)
        set_props = getattr(self._dc, "set_articulation_dof_properties", None)
        if get_props is None or set_props is None:
            _emit_bridge_log(
                f"[WARN] Dynamic Control DOF property API is unavailable. Lift drive tuning was skipped for {namespace}."
            )
            return

        try:
            props = get_props(art)
            if props is None:
                _emit_bridge_log(f"[WARN] Lift DOF properties unavailable for {namespace}.")
                return

            field_names = getattr(getattr(props, "dtype", None), "names", None) or ()
            if "stiffness" in field_names:
                props["stiffness"][dof_index] = float(LIFT_DRIVE_STIFFNESS)
            if "damping" in field_names:
                props["damping"][dof_index] = float(LIFT_DRIVE_DAMPING)
            if "maxEffort" in field_names:
                props["maxEffort"][dof_index] = float(LIFT_DRIVE_MAX_FORCE)
            if "maxForce" in field_names:
                props["maxForce"][dof_index] = float(LIFT_DRIVE_MAX_FORCE)

            set_props(art, props)
            _emit_bridge_log(
                "[INFO] Tuned %s lift drive: dof_index=%d max_force=%.1f stiffness=%.1f damping=%.1f"
                % (
                    namespace,
                    dof_index,
                    LIFT_DRIVE_MAX_FORCE,
                    LIFT_DRIVE_STIFFNESS,
                    LIFT_DRIVE_DAMPING,
                )
            )
        except Exception as exc:  # noqa: BLE001
            _emit_bridge_log(f"[WARN] Failed to tune lift drive for {namespace}: {exc}")

    def _ensure_high_friction_physics_material(self):
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None

        material_prim = stage.DefinePrim(PHYSICS_MATERIAL_PRIM_PATH, "Material")
        physics_api = UsdPhysics.MaterialAPI.Apply(material_prim)
        physics_api.CreateStaticFrictionAttr().Set(float(HIGH_FRICTION_VALUE))
        physics_api.CreateDynamicFrictionAttr().Set(float(HIGH_FRICTION_VALUE))
        physics_api.CreateRestitutionAttr().Set(0.0)

        if PhysxSchema is not None:
            try:
                physx_api = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
                physx_api.CreateImprovePatchFrictionAttr().Set(True)
            except Exception as exc:  # noqa: BLE001
                print(f"[WARN] Failed to apply PhysX material API: {exc}")

        return UsdShade.Material.Get(stage, PHYSICS_MATERIAL_PRIM_PATH)

    def _iter_collision_prims(self, root_prim):
        stack = [root_prim]
        while stack:
            prim = stack.pop()
            if not prim or not prim.IsValid():
                continue
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                yield prim
            stack.extend(reversed(list(prim.GetChildren())))

    def _bind_physics_material(self, prim, material) -> bool:
        try:
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.Bind(
                material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics",
            )
            return True
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] Failed to bind physics material on {prim.GetPath()}: {exc}")
            return False

    def _apply_high_friction_material(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            print("[WARN] USD stage unavailable. High-friction physics material was not applied.")
            return

        material = self._ensure_high_friction_physics_material()
        if material is None or not material:
            print("[WARN] High-friction physics material could not be created.")
            return

        target_roots = []
        for prim_path in LIFT_CONTACT_PRIM_PATHS:
            prim = stage.GetPrimAtPath(prim_path)
            if prim and prim.IsValid():
                target_roots.append(prim)
            else:
                print(f"[WARN] Lift contact prim not found for high-friction binding: {prim_path}")

        for prim in stage.Traverse():
            prim_name = prim.GetName()
            if prim_name.startswith(PALLET_PRIM_PREFIX):
                target_roots.append(prim)

        applied_count = 0
        applied_paths = set()
        for root in target_roots:
            for prim in self._iter_collision_prims(root):
                prim_path = prim.GetPath().pathString
                if prim_path in applied_paths:
                    continue
                if self._bind_physics_material(prim, material):
                    applied_paths.add(prim_path)
                    applied_count += 1

        print(
            "[INFO] Applied high-friction physics material to %d collision prim(s) under lift contact prims and pallet bins."
            % applied_count
        )

    def _get_target_visual_mtime(self):
        try:
            return os.path.getmtime(TARGET_VISUAL_CMD_PATH)
        except OSError:
            return None

    def _log_target_visual_watch_status(self) -> None:
        if self._last_target_visual_mtime is None:
            print(
                f"[INFO] Target visual bridge connected. Waiting for planner to create {TARGET_VISUAL_CMD_PATH}."
            )
        else:
            print(
                "[INFO] Target visual bridge connected. Existing target command is ignored until planner writes a fresh update."
            )

    def _read_cmd(self) -> None:
        try:
            with open(LIFT_CMD_PATH, "r", encoding="utf-8") as handle:
                cmds = json.load(handle)
            for namespace in self._targets:
                if namespace in cmds:
                    target = float(cmds[namespace])
                    if abs(self._targets[namespace] - target) > 1e-6:
                        self._ramp_start_positions[namespace] = self._get_current_position(namespace)
                        self._ramp_start_times[namespace] = self._sim_time
                    self._targets[namespace] = target
                    if self._last_logged_targets.get(namespace) != target:
                        self._last_logged_targets[namespace] = target
                        print(f"[INFO] lift_cmd received: {namespace} -> {target:.4f}")
        except (FileNotFoundError, json.JSONDecodeError, ValueError):
            pass

    def _get_current_position(self, namespace: str) -> float:
        dof_handle = self._joints.get(namespace)
        if dof_handle is None or self._dc is None:
            return self._applied_targets.get(namespace, 0.0)
        try:
            return float(self._dc.get_dof_position(dof_handle))
        except Exception:
            return self._applied_targets.get(namespace, 0.0)

    def _get_ramped_target(self, namespace: str) -> float:
        commanded_target = self._targets.get(namespace, 0.0)
        start_position = self._ramp_start_positions.get(namespace, 0.0)
        start_time = self._ramp_start_times.get(namespace, 0.0)
        elapsed = max(0.0, self._sim_time - start_time)
        if LIFT_RAMP_DURATION_SEC <= 0.0 or elapsed >= LIFT_RAMP_DURATION_SEC:
            return commanded_target
        alpha = elapsed / LIFT_RAMP_DURATION_SEC
        return start_position + ((commanded_target - start_position) * alpha)

    def _write_state(self, states: dict) -> None:
        try:
            state_dir = os.path.dirname(LIFT_STATE_PATH)
            if state_dir:
                os.makedirs(state_dir, exist_ok=True)
            with open(LIFT_STATE_PATH, "w", encoding="utf-8") as handle:
                json.dump(states, handle)
        except Exception:
            pass

    def _write_cmd(self, cmds: dict) -> None:
        try:
            cmd_dir = os.path.dirname(LIFT_CMD_PATH)
            if cmd_dir:
                os.makedirs(cmd_dir, exist_ok=True)
            with open(LIFT_CMD_PATH, "w", encoding="utf-8") as handle:
                json.dump(cmds, handle)
        except Exception:
            pass

    def _iter_highlight_prims(self, root_prim):
        stack = [root_prim]
        while stack:
            prim = stack.pop()
            if not prim or not prim.IsValid():
                continue
            if prim.IsA(UsdGeom.Gprim):
                yield prim
            stack.extend(reversed(list(prim.GetChildren())))

    def _ensure_target_highlight_material(self, color_rgb):
        stage = omni.usd.get_context().get_stage()
        looks_scope = stage.DefinePrim("/Root/Looks", "Scope")
        if not looks_scope or not looks_scope.IsValid():
            return None

        material = UsdShade.Material.Define(stage, "/Root/Looks/TargetPalletHighlight")
        shader = UsdShade.Shader.Define(
            stage, "/Root/Looks/TargetPalletHighlight/PreviewSurface"
        )
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(float(color_rgb[0]), float(color_rgb[1]), float(color_rgb[2]))
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    def _is_yellow_display_color(self, authored_value) -> bool:
        try:
            if authored_value is None or len(authored_value) != 1:
                return False
            color = authored_value[0]
            return (
                abs(float(color[0]) - 1.0) < 1e-4
                and abs(float(color[1]) - 1.0) < 1e-4
                and abs(float(color[2]) - 0.0) < 1e-4
            )
        except Exception:
            return False

    def _clear_stale_target_highlights(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        highlight_material_path = "/Root/Looks/TargetPalletHighlight"
        cleared_material_bindings = 0
        cleared_display_colors = 0

        root = stage.GetPrimAtPath("/Root")
        if not root or not root.IsValid():
            return

        for pallet_root in root.GetChildren():
            if not pallet_root or not pallet_root.IsValid():
                continue
            if not pallet_root.GetName().startswith("PalletBin_"):
                continue

            root_binding_api = UsdShade.MaterialBindingAPI(pallet_root)
            root_targets = list(root_binding_api.GetDirectBindingRel().GetTargets())
            if any(target.pathString == highlight_material_path for target in root_targets):
                root_binding_api.GetDirectBindingRel().ClearTargets(True)
                cleared_material_bindings += 1

            for prim in self._iter_highlight_prims(pallet_root):
                binding_api = UsdShade.MaterialBindingAPI(prim)
                targets = list(binding_api.GetDirectBindingRel().GetTargets())
                if any(target.pathString == highlight_material_path for target in targets):
                    binding_api.GetDirectBindingRel().ClearTargets(True)
                    cleared_material_bindings += 1

                gprim = UsdGeom.Gprim(prim)
                attr = gprim.GetDisplayColorAttr()
                if attr and attr.HasAuthoredValueOpinion():
                    authored_value = attr.Get()
                    if self._is_yellow_display_color(authored_value):
                        attr.Clear()
                        cleared_display_colors += 1

        if cleared_material_bindings or cleared_display_colors:
            print(
                "[INFO] Cleared stale target highlight artifacts: "
                f"materials={cleared_material_bindings}, display_colors={cleared_display_colors}"
            )

    def _restore_previous_target_highlight(self) -> None:
        if not self._highlight_original_colors:
            if self._highlighted_target_prim and self._highlight_root_original_material_targets is not None:
                stage = omni.usd.get_context().get_stage()
                target_prim = stage.GetPrimAtPath(self._highlighted_target_prim)
                if target_prim and target_prim.IsValid():
                    binding_api = UsdShade.MaterialBindingAPI(target_prim)
                    direct_binding_rel = binding_api.GetDirectBindingRel()
                    if self._highlight_root_original_material_targets:
                        direct_binding_rel.SetTargets(self._highlight_root_original_material_targets)
                    else:
                        direct_binding_rel.ClearTargets(True)
            self._highlighted_target_prim = None
            self._highlight_root_original_material_targets = None
            return

        stage = omni.usd.get_context().get_stage()
        if self._highlighted_target_prim:
            target_prim = stage.GetPrimAtPath(self._highlighted_target_prim)
            if target_prim and target_prim.IsValid():
                binding_api = UsdShade.MaterialBindingAPI(target_prim)
                direct_binding_rel = binding_api.GetDirectBindingRel()
                if self._highlight_root_original_material_targets:
                    direct_binding_rel.SetTargets(self._highlight_root_original_material_targets)
                else:
                    direct_binding_rel.ClearTargets(True)

        for prim_path, original_value in self._highlight_original_colors.items():
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                continue
            gprim = UsdGeom.Gprim(prim)
            attr = gprim.GetDisplayColorAttr()
            if original_value is None:
                if attr:
                    attr.Clear()
            else:
                attr.Set(original_value)

            binding_api = UsdShade.MaterialBindingAPI(prim)
            direct_binding_rel = binding_api.GetDirectBindingRel()
            original_targets = self._highlight_original_material_targets.get(prim_path)
            if original_targets:
                direct_binding_rel.SetTargets(original_targets)
            else:
                direct_binding_rel.ClearTargets(True)

        self._highlight_original_colors = {}
        self._highlight_original_material_targets = {}
        self._highlighted_target_prim = None
        self._highlight_root_original_material_targets = None

    def _apply_target_highlight(self, target_prim_path: str, color_rgb, force_log: bool = False) -> None:
        if target_prim_path == self._highlighted_target_prim:
            if force_log and target_prim_path:
                print(
                    f"[INFO] Target visual bridge already connected to highlighted prim {target_prim_path}."
                )
            return

        self._restore_previous_target_highlight()
        self._clear_stale_target_highlights()

        if not target_prim_path:
            if force_log:
                print("[INFO] Target visual bridge connected, but no target pallet has been requested yet.")
            return

        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(target_prim_path)
        if not target_prim or not target_prim.IsValid():
            print(f"[WARN] Target highlight prim not found: {target_prim_path}")
            return

        highlight_material = self._ensure_target_highlight_material(color_rgb)
        highlight_color = [Gf.Vec3f(float(color_rgb[0]), float(color_rgb[1]), float(color_rgb[2]))]

        if highlight_material is not None:
            try:
                root_binding_api = UsdShade.MaterialBindingAPI(target_prim)
                self._highlight_root_original_material_targets = list(
                    root_binding_api.GetDirectBindingRel().GetTargets()
                )
                root_binding_api.Bind(
                    highlight_material,
                    bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"[WARN] Failed to bind highlight material on root {target_prim_path}: {exc}")

        mesh_count = 0
        for prim in self._iter_highlight_prims(target_prim):
            gprim = UsdGeom.Gprim(prim)
            attr = gprim.GetDisplayColorAttr()
            original_value = attr.Get() if attr and attr.HasAuthoredValueOpinion() else None
            prim_path = prim.GetPath().pathString
            self._highlight_original_colors[prim_path] = original_value
            if not attr:
                attr = gprim.CreateDisplayColorAttr()
            attr.Set(highlight_color)

            binding_api = UsdShade.MaterialBindingAPI(prim)
            direct_binding_rel = binding_api.GetDirectBindingRel()
            self._highlight_original_material_targets[prim_path] = list(
                direct_binding_rel.GetTargets()
            )
            if highlight_material is not None:
                binding_api.Bind(highlight_material)
            mesh_count += 1

        self._highlighted_target_prim = target_prim_path
        print(
            f"[INFO] Highlighted target pallet {target_prim_path} in yellow across {mesh_count} prim(s)."
        )

    def _read_target_visual_cmd(self, force_log: bool = False) -> None:
        current_mtime = self._get_target_visual_mtime()
        if not force_log and current_mtime == self._last_target_visual_mtime:
            return

        try:
            with open(TARGET_VISUAL_CMD_PATH, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
        except (FileNotFoundError, json.JSONDecodeError, ValueError):
            if force_log:
                print(
                    f"[INFO] Target visual bridge connected, but {TARGET_VISUAL_CMD_PATH} is not ready yet."
                )
            return

        self._last_target_visual_mtime = current_mtime

        target_prim_path = str(payload.get("target_prim_path", "")).strip()
        color_rgb = payload.get("display_color_rgb", [1.0, 1.0, 0.0])
        if not isinstance(color_rgb, list) or len(color_rgb) != 3:
            color_rgb = [1.0, 1.0, 0.0]

        cmd_signature = (target_prim_path, tuple(float(v) for v in color_rgb))
        if force_log or cmd_signature != self._last_target_visual_cmd:
            self._last_target_visual_cmd = cmd_signature
            if target_prim_path:
                print(
                    f"[INFO] Target visual command received: prim={target_prim_path}, color={color_rgb}"
                )
            else:
                print("[INFO] Target visual command received: no active target prim.")

        self._apply_target_highlight(target_prim_path, color_rgb, force_log=force_log)

    def _on_physics_step(self, _dt: float) -> None:
        if self._dc is None:
            return

        self._sim_time += float(_dt)
        self._step_count += 1
        if self._step_count % POLL_INTERVAL == 0:
            self._read_cmd()
            self._read_target_visual_cmd()

        states = {}
        for namespace, dof_handle in self._joints.items():
            target = self._get_ramped_target(namespace)
            self._applied_targets[namespace] = target
            art = self._articulations.get(namespace)
            if art:
                self._dc.wake_up_articulation(art)
            self._dc.set_dof_position_target(dof_handle, target)
            try:
                current = float(self._dc.get_dof_position(dof_handle))
            except Exception:
                current = 0.0

            if abs(current - target) > 1e-4:
                try:
                    self._dc.set_dof_position(dof_handle, target)
                    current = float(self._dc.get_dof_position(dof_handle))
                except Exception:
                    pass

            try:
                states[namespace] = round(float(current), 6)
            except Exception:
                states[namespace] = 0.0

        if self._step_count % POLL_INTERVAL == 0 and states:
            self._write_state(states)


_LIFT_BRIDGE = None


def start_lift_bridge():
    global _LIFT_BRIDGE
    if _LIFT_BRIDGE is not None:
        try:
            _LIFT_BRIDGE.shutdown()
        except Exception:  # noqa: BLE001
            pass
    _LIFT_BRIDGE = LiftBridgeRuntime()
    return _LIFT_BRIDGE


start_lift_bridge()
