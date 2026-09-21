"""Browser renderer implementing the same lifecycle as MjviewerRenderer."""

import socket

import numpy as np


class MjviserRenderer:
    def __init__(self, env, host="127.0.0.1", port=8080):
        self.env = env
        self.host = host
        self.port = port
        self.server = None
        self.scene = None
        self._model = None
        self._camera_position = None
        self._camera_target = None

    def _frame_client(self, client):
        if self._camera_target is not None:
            client.camera.up_direction = (0, 0, 1)
            client.camera.fov = np.deg2rad(60)
            client.camera.position = self._camera_position
            client.camera.look_at = self._camera_target

    def _frame_robot(self):
        robot = self.env.robots[0].robot_model
        # A mobile robot's MJCF root can remain at a staging position while
        # planar joints move its chassis into the kitchen.
        center = robot.base.correct_naming("center") if robot.base is not None else None
        if center in self.env.sim.model.site_names:
            position = self.env.sim.data.get_site_xpos(center).copy()
            rotation = self.env.sim.data.get_site_xmat(center)
        else:
            position = self.env.sim.data.get_body_xpos(robot.root_body).copy()
            rotation = self.env.sim.data.get_body_xmat(robot.root_body)
        self._camera_target = position + np.array([0, 0, 0.8])
        self._camera_position = self._camera_target + rotation @ np.array([-3, -3, 2])
        for client in self.server.get_clients().values():
            self._frame_client(client)

    def update(self):
        # Import and start lazily: constructing a headless environment must not
        # open a socket, and the model may change before the first render.
        import viser
        from mjviser import ViserMujocoScene

        if self.server is None:
            # Viser waits for its server thread; surface sandbox errors here.
            with socket.socket() as probe:
                probe.bind((self.host, 0))
            self.server = viser.ViserServer(host=self.host, port=self.port)
            self.server.on_client_connect(self._frame_client)
        model = self.env.sim.model._model
        if self._model is not model:
            self.server.scene.reset()
            self.server.gui.reset()
            loading = self.server.gui.add_markdown("Loading environment meshes…")
            print("Building mjviser scene; large kitchens can take a moment.", flush=True)
            self.scene = ViserMujocoScene(self.server, model, num_envs=1)
            # Match mjviewer's visual-only default. Collision shells can enclose
            # the entire kitchen, hiding its contents in the browser.
            self.scene.geom_groups_visible[0] = False
            self.scene.site_groups_visible = [False] * 6
            self.scene.camera_tracking_enabled = False
            self.scene._sync_visibilities()
            # MuJoCo's global bounds include distant scenery. Do not use the
            # upstream 3 * model.stat.extent camera distance (80m for layout 1).
            tabs = self.server.gui.add_tab_group()
            with tabs.add_tab("Scene"):
                frame = self.server.gui.add_button("Frame robot")
                frame.on_click(lambda _: self._frame_robot())
            with tabs.add_tab("Visualization"):
                self.scene.create_overlay_gui()
            with tabs.add_tab("Groups"):
                self.scene.create_groups_gui()
            loading.remove()
            self._frame_robot()
            self._model = model
        self.scene.update_from_mjdata(self.env.sim.data._data)

    def render(self):
        self.update()

    def reset(self):
        # Rebuild lazily after wrappers have applied visual changes. Retain the
        # server across hard resets so connected browsers keep the same URL.
        self._model = None
        self.scene = None

    def close(self):
        if self.server is not None:
            self.server.stop()
            self.server = None
        self.reset()
