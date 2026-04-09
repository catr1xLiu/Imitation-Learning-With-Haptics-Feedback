import sys
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import time

class RobotArmGUI:
    def __init__(self, ctrl, points, oris, update_hz=10, extra_point_sets=None, gripper=None):
        self.ctrl, self.points, self.oris, self.gripper = ctrl, points, oris, gripper
        self.update_hz = update_hz
        self.idx, self.history = 0, []
        self.lim = 0.4
        self.jog_delta = None

        self.extra_point_sets = extra_point_sets or []
        self.extra_colors = ['red', 'green', 'purple', 'orange', 'cyan', 'magenta']

        self.xmin = self.ctrl.xmin
        self.xmax = self.ctrl.xmax
        self.ymin = self.ctrl.ymin
        self.ymax = self.ctrl.ymax
        self.zmin = self.ctrl.zmin
        self.zmax = self.ctrl.zmax

        self.start_pos = np.array(points[0])
        self.end_pos = np.array(points[-1])

        self.iso_views = [(30, 315), (30, 45), (30, 135), (30, 225)]
        self.iso_idx = -1

        self.ctrl.paused = False       
        self.rendering_paused = True    # Pause rendering by default

        self.root = tk.Tk()
        self._update_title()
        self.root.configure(bg="#ffffff")
        self.root.geometry("1000x800")
        self.root.bind_all("<Key>", self._on_key_press)

        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background="#ffffff")
        style.configure("TLabel", background="#ffffff", font=("Poppins", 12))
        style.configure("Control.TLabelframe", background="#f8f8f8", relief="flat", borderwidth=0)
        style.configure("Control.TLabelframe.Label", font=("Poppins", 14, "bold"), foreground="#444")
        style.configure("Accent.TButton", font=("Poppins", 14), padding=10, relief="flat", borderwidth=0)
        style.configure("Stop.TButton", font=("Poppins", 14, "bold"), background="#e74c3c", foreground="white")
        style.map("Stop.TButton", background=[("active", "#c0392b")])
        style.configure("Pause.TButton", font=("Poppins", 14, "bold"), background="#f39c12", foreground="white")
        style.map("Pause.TButton", background=[("active", "#e67e22")])

        container = ttk.Frame(self.root, padding=20)
        container.pack(fill="both", expand=True)

        control_row = ttk.Frame(container)
        control_row.pack(side="top", fill="x", padx=(0, 10))
        self._build_controls(control_row)

        plot_row = ttk.Frame(container)
        plot_row.pack(side="bottom", fill="both", expand=True)
        self._build_plot(plot_row)
        self._zoom(1.1)
        self._cycle_iso()
        self._cycle_iso()
        self._cycle_iso()

        self._gui_loop()

        self.root.mainloop()

    def update(self, points, oris, gripper):
        self.points = points
        self.oris = oris
        self.gripper = gripper
        self.idx = 0
        self.history = []

        # Clear plot
        self.line.set_data([], [])
        self.line.set_3d_properties([])

        # Start loop again
        self._gui_loop()

    
    def _update_title(self):
        if self.ctrl.paused:
            self.root.title("🤖 Robot Arm Controller - PAUSED (Press SPACE to Resume)")
        else:
            self.root.title("🤖 Robot Arm Controller - RUNNING (Press SPACE to Pause)")

    def _on_key_press(self, event):
        if event.keysym == 'space':
            self._toggle_pause()

    def _build_controls(self, parent):
        col1 = ttk.Frame(parent)
        col1.pack(side="left", fill="y", expand=True, padx=10)

        readout = ttk.Labelframe(col1, text="📊 Feedback", style="Control.TLabelframe", padding=15)
        readout.pack(fill="both", pady=10)

        self.d_labels = []
        self.a_labels = []

        # 👉 Feedback readout loop
        for i in range(6):
            row = ttk.Frame(readout)
            row.pack(fill="x", pady=5)
            ttk.Label(row, text=f"Δ{i + 1}", width=6, anchor="w").pack(side="left", padx=5)
            dl = ttk.Label(row, text="0.000", width=8, anchor="w")
            dl.pack(side="left")
            self.d_labels.append(dl)

            ttk.Label(row, text=f"P/O{i + 1}", width=6, anchor="w").pack(side="left", padx=5)
            al = ttk.Label(row, text="0.000", width=8, anchor="w")
            al.pack(side="left")
            self.a_labels.append(al)

        # ✅ Now add buttons *after* the loop
        sp = ttk.Frame(col1)
        sp.pack(fill="x", pady=10)

        ttk.Button(sp, text="⏹ Stop", style="Stop.TButton", command=self._kill_immediately).pack(fill="x", pady=5)

        self.robot_pause_btn = ttk.Button(sp, text="▶ Resume Robot", style="Pause.TButton", command=self._toggle_robot_pause)
        self.robot_pause_btn.pack(fill="x", pady=5)

        self.render_pause_btn = ttk.Button(sp, text="▶ Resume Rendering", style="Pause.TButton", command=self._toggle_render_pause)
        self.render_pause_btn.pack(fill="x", pady=5)


        ttk.Button(sp, text="🔁 Reset Path", style="Accent.TButton", command=self._reset_path).pack(fill="x", pady=5)




        col2 = ttk.Frame(parent)
        col2.pack(side="left", fill="y", expand=True, padx=10)
        jog = ttk.Labelframe(col2, text="🤏 Jog Controls", style="Control.TLabelframe", padding=15)
        jog.pack(fill="both", pady=10)
        moves = [("X+", [0.15, 0, 0]), ("X-", [-0.15, 0, 0]), ("Y+", [0, 0.15, 0]),
                 ("Y-", [0, -0.15, 0]), ("Z+", [0, 0, 0.015]), ("Z-", [0, 0, -0.015])]
        for lbl, delta in moves:
            btn = ttk.Button(jog, text=lbl, style="Accent.TButton")
            btn.pack(fill="x", pady=5)
            btn.bind("<ButtonPress-1>", lambda e, d=delta: self._start_jog(d))
            btn.bind("<ButtonRelease-1>", lambda e: self._stop_jog())

        col3 = ttk.Frame(parent)
        col3.pack(side="left", fill="y", expand=True, padx=10)
        vz = ttk.Labelframe(col3, text="🔍 View & Zoom", style="Control.TLabelframe", padding=15)
        vz.pack(fill="both", pady=10)
        for txt, cmd in [
            ("Iso", self._cycle_iso),
            ("X-flat", lambda: self.ax.view_init(90, 0)),
            ("Y-flat", lambda: self.ax.view_init(90, 90)),
            ("Z-flat", lambda: self.ax.view_init(0, 90)),
            ("Zoom In", lambda: self._zoom(0.8)),
            ("Zoom Out", lambda: self._zoom(1.25))
        ]:
            ttk.Button(vz, text=txt, style="Accent.TButton", command=cmd).pack(fill="x", pady=5)

    def _build_plot(self, parent):
        self.fig = plt.Figure(figsize=(6, 5), tight_layout=True)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.view_init(elev=20, azim=45)
        self._apply_limits()
        self.ax.set_xlabel("X (m)", fontsize=12)
        self.ax.set_ylabel("Y (m)", fontsize=12)
        self.ax.set_zlabel("Z (m)", fontsize=12)

        x_plane = np.array([[self.ctrl.xmin, self.ctrl.xmin], [self.ctrl.xmax, self.ctrl.xmax]])
        y_plane = np.array([[self.ctrl.ymin, self.ctrl.ymin], [self.ctrl.ymax, self.ctrl.ymax]])
        z_plane = np.array([[self.ctrl.zmin, self.ctrl.zmin], [self.ctrl.zmax, self.ctrl.zmax]])

        self.ax.plot_surface(x_plane, y_plane, z_plane, color='saddlebrown', alpha=0.2)
        self.ax.plot_surface(x_plane, y_plane, z_plane + 0.1, color='saddlebrown', alpha=0.2)

        self._draw_box(self.start_pos[:2])
        self._draw_box(self.end_pos[:2])

        for i, point_set in enumerate(self.extra_point_sets):
            pts = np.array(point_set)
            color = self.extra_colors[i % len(self.extra_colors)]

            if i == 0:
                self.ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=color, label=f"Extra Set {i + 1}")
            elif i == 1:
                self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=color, linewidth=1.5, label=f"Extra Set {i + 1}")

        self.line, = self.ax.plot([], [], [], "o-", lw=2, markersize=6, markerfacecolor="#3498db")
        canvas = FigureCanvasTkAgg(self.fig, parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas = canvas

    def _draw_box(self, xy, size=0.1, color='red'):
        x, y = xy
        d = size / 2
        cz = 0
        verts = np.array([[x - d, y - d, cz], [x + d, y - d, cz], [x + d, y + d, cz], [x - d, y + d, cz],
                          [x - d, y - d, cz + size], [x + d, y - d, cz + size], [x + d, y + d, cz + size],
                          [x - d, y + d, cz + size]])
        faces = [[verts[j] for j in f] for f in [
            [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
            [2, 3, 7, 6], [1, 2, 6, 5], [4, 7, 3, 0]
        ]]
        self.ax.add_collection3d(Poly3DCollection(faces, facecolors=color, edgecolors='white', alpha=0.99))

    def _apply_limits(self):
        self.ax.set_xlim(self.xmin, self.xmax)
        self.ax.set_ylim(self.ymin, self.ymax)
        self.ax.set_zlim(self.zmin, self.zmax)

    def _toggle_robot_pause(self):
        self.ctrl.paused = not self.ctrl.paused
        self.robot_pause_btn.config(text="▶ Resume Robot" if self.ctrl.paused else "⏸ Pause Robot")
        self._update_title()

    def _toggle_render_pause(self):
        self.rendering_paused = not self.rendering_paused
        self.render_pause_btn.config(text="▶ Resume Rendering" if self.rendering_paused else "⏸ Pause Rendering")


    def _start_jog(self, delta):
        self.jog_delta = delta
        self._do_jog()

    def _stop_jog(self):
        self.jog_delta = None

    def _do_jog(self):
        if not self.jog_delta:
            return
        self.ctrl.jog(self.jog_delta)
        d, a = self.ctrl.get_position_and_orientation()
        for i in range(6):
            self.d_labels[i].config(text=f"{d[i]:.3f}")
            self.a_labels[i].config(text=f"{a[i]:.3f}")
        self.history.append(a)
        pts = np.array(self.history)
        self.line.set_data(pts[:, 0], pts[:, 1])
        self.line.set_3d_properties(pts[:, 2])
        self.canvas.draw()
        self.root.after(100, self._do_jog)

    def _zoom(self, factor):
        self.xmin *= factor
        self.xmax *= factor
        self.ymin *= factor
        self.ymax *= factor
        self.zmin *= factor
        self.zmax *= factor
        self._apply_limits()
        self.canvas.draw()

    def _cycle_iso(self):
        self.iso_idx = (self.iso_idx + 1) % len(self.iso_views)
        elev, azim = self.iso_views[self.iso_idx]
        self.ax.view_init(elev, azim)
        self.canvas.draw()

    def _gui_loop(self):
        if self.idx >= len(self.points):
            return

        if not self.ctrl.paused:
            p, o, g = self.points[self.idx], self.oris[self.idx], self.gripper[self.idx]
            self.ctrl.move_to_delta(p, o, g)

            if not self.rendering_paused:
                d, a = self.ctrl.get_position_and_orientation()
                for i in range(6):
                    self.d_labels[i].config(text=f"{d[i]:.3f}")
                    self.a_labels[i].config(text=f"{a[i]:.3f}")

                self.history.append(a)
                pts = np.array(self.history)
                self.line.set_data(pts[:, 0], pts[:, 1])
                self.line.set_3d_properties(pts[:, 2])
                self.canvas.draw()

            self.idx += 1

        if self.idx < len(self.points):
            self.root.after(int(1000 / (self.update_hz * 20)), self._gui_loop)
        else:
            self.ctrl.env.save_data()
            print("Task sucess, stopping")
            self.root.destroy()

    def _kill_immediately(self):
        self.ctrl.emergency_stop()
        # self.ctrl.shutdown()
        # self.root.destroy()
        sys.exit(0)

    def _reset_path(self):
        self.idx = 0
        self.history = []
        self.line.set_data([], [])
        self.line.set_3d_properties([])
        if not self.rendering_paused:
            self.canvas.draw()

