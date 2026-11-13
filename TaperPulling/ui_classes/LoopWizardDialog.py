# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel / Assistant
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
Interactive Loop Wizard Dialog for Fiber Taper Loop Fabrication.
Provides real-time position telemetry, micro-step jogging, press-and-hold
continuous slow creeping, and step-by-step guidance.
If a tunable laser is available, transmission spectra can be measured at any
moment. Each one is saved with the motor positions at the time.
A camera can be shown, to watch the loop. Snapshots and videos can be saved,
and a snapshot is saved with every spectrum.
"""

import os
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QDoubleSpinBox, QButtonGroup,
    QMessageBox, QTextEdit, QLineEdit, QProgressBar, QComboBox, QSpinBox, QCheckBox,
    QSizePolicy
)
from PyQt6.QtGui import QFont, QFontMetrics, QImage, QPixmap
from datetime import datetime


class LoopWizardDialog(QDialog):
    """
    Interactive Dialog for guiding the fabrication and tuning of fiber taper loops.
    """

    def __init__(self, parent=None, core=None, main=None):
        """
        Args:
            core: TaperPullingCore object (motors).
            main: Main window. If given, it provides the tunable laser, the data
                acquisition and the sweep settings, and the spectrum panel is shown.
        """
        super().__init__(parent)
        self.core = core
        self.main = main
        self.spectrum = main.spectrum if main is not None else None
        self.camera = main.camera if main is not None else None
        self.camera_frame_count = -1
        self.setWindowTitle("Fiber Taper Loop Wizard")
        self.setMinimumSize(600, 700)
        self.resize(1680 if self.spectrum is not None else 640, 730)
        
        # Spectra measured in this session
        self.spectra = []          # (wavelength, power, slack)
        self.spectra_folder = ""   # Created when the first spectrum is saved

        # Store initial positions for delta tracking and reset functionality
        self.left_motor = self.core.motors.left_puller if self.core else None
        self.right_motor = self.core.motors.right_puller if self.core else None

        self.init_left_pos = self.left_motor.get_position() if self.left_motor else 0.0
        self.init_right_pos = self.right_motor.get_position() if self.right_motor else 0.0

        # Puller positions where the taper was straight (end of the pull, or after tensioning
        # it), if known: the slack measured from them does not restart when the wizard is
        # opened again. See MainWindow.get_slack_reference()
        self.slack_ref = None
        if main is not None and hasattr(main, "get_slack_reference"):
            self.slack_ref = main.get_slack_reference()
        self.pull_end_pos = self.slack_ref["reference"] if self.slack_ref is not None else None

        # Save default velocities to restore after creeping
        self.default_left_vel = self.left_motor.vel if self.left_motor else 40.0
        self.default_right_vel = self.right_motor.vel if self.right_motor else 40.0
        self.is_holding_continuous = False

        # Current step (0: Loosen, 1: Twist, 2: Tension)
        self.current_step = 0

        self.init_ui()

        # Camera image update timer
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.setInterval(50)
        if self.camera is not None and self.main.cameraAutoCheck.isChecked():
            self.btn_camera.setChecked(True)
            self.toggle_camera()

        # Sweep update timer
        self.sweep_timer = QTimer(self)
        self.sweep_timer.timeout.connect(self.update_sweep)
        self.sweep_timer.setInterval(20)

        # Telemetry update timer (50 ms / 20 Hz)
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        self.telemetry_timer.start(50)

    def init_ui(self):
        # Motor controls on the left, spectrum panel on the right
        outer_layout = QHBoxLayout(self)
        outer_layout.setContentsMargins(15, 15, 15, 15)
        outer_layout.setSpacing(12)
        self.motors_panel = QWidget(self)
        self.motors_panel.setMaximumWidth(640)
        outer_layout.addWidget(self.motors_panel)
        if self.spectrum is not None:
            outer_layout.addWidget(self.init_spectrum_panel(), 3)
            outer_layout.addWidget(self.init_camera_panel(), 2)
        
        main_layout = QVBoxLayout(self.motors_panel)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(0, 0, 0, 0)

        step_btn_stylesheet = """
        QPushButton {
            background-color: #e2e8f0;
            color: #1e293b;
            border: 1px solid #cbd5e1;
            border-radius: 4px;
            padding: 7px;
            font-weight: 500;
        }
        QPushButton:checked {
            background-color: #2563eb;
            color: #ffffff;
            font-weight: bold;
            border: 1px solid #1d4ed8;
        }
        QPushButton:hover:!checked {
            background-color: #cbd5e1;
        }
        """

        preset_btn_stylesheet = """
        QPushButton {
            background-color: #f1f5f9;
            color: #0f172a;
            border: 1px solid #cbd5e1;
            border-radius: 3px;
            padding: 4px 6px;
            font-weight: 500;
        }
        QPushButton:hover {
            background-color: #e2e8f0;
            border-color: #94a3b8;
        }
        """

        # --- 1. Step-by-Step Guide Panel ---
        guide_group = QGroupBox("Fabrication Guide", self)
        guide_layout = QVBoxLayout(guide_group)

        # Step indicator header
        step_header_layout = QHBoxLayout()
        self.step_btn1 = QPushButton("1. Loosen (FW)", self)
        self.step_btn2 = QPushButton("2. Twist / Loop", self)
        self.step_btn3 = QPushButton("3. Tension (BW)", self)

        for btn in (self.step_btn1, self.step_btn2, self.step_btn3):
            btn.setCheckable(True)
            btn.setStyleSheet(step_btn_stylesheet)
            step_header_layout.addWidget(btn)

        self.step_btn_group = QButtonGroup(self)
        self.step_btn_group.addButton(self.step_btn1, 0)
        self.step_btn_group.addButton(self.step_btn2, 1)
        self.step_btn_group.addButton(self.step_btn3, 2)
        self.step_btn_group.idClicked.connect(self.set_step)
        self.step_btn1.setChecked(True)
        guide_layout.addLayout(step_header_layout)

        # Step instruction text
        self.guide_text = QTextEdit(self)
        self.guide_text.setReadOnly(True)
        self.guide_text.setMaximumHeight(85)
        self.guide_text.setStyleSheet(
            "background-color: #f8fafc; color: #0f172a; border: 1px solid #cbd5e1; border-radius: 4px; padding: 6px; font-size: 13px;"
        )
        guide_layout.addWidget(self.guide_text)

        main_layout.addWidget(guide_group)

        # --- 2. Live Position & Telemetry Panel ---
        pos_group = QGroupBox("Live Telemetry", self)
        pos_layout = QGridLayout(pos_group)
        pos_layout.setSpacing(8)

        bold_font = QFont()
        bold_font.setBold(True)

        mono_font = QFont("Monospace")
        mono_font.setStyleHint(QFont.StyleHint.TypeWriter)
        mono_font.setPixelSize(13)
        mono_bold = QFont(mono_font)
        mono_bold.setBold(True)
        fm = QFontMetrics(mono_bold)

        header_motor = QLabel("Motor")
        header_pos = QLabel("Current Position")
        header_delta = QLabel("Displacement (Δ)")
        header_status = QLabel("Status")
        for lbl in (header_motor, header_pos, header_delta, header_status):
            lbl.setFont(bold_font)

        pos_layout.addWidget(header_motor, 0, 0)
        pos_layout.addWidget(header_pos, 0, 1)
        pos_layout.addWidget(header_delta, 0, 2)
        pos_layout.addWidget(header_status, 0, 3)

        # Left Puller row
        pos_layout.addWidget(QLabel("Left Puller:"), 1, 0)
        self.lbl_left_pos = QLabel("0.000 mm")
        self.lbl_left_pos.setFont(mono_font)
        self.lbl_left_delta = QLabel("+0.000 mm")
        self.lbl_left_delta.setFont(mono_font)
        self.lbl_left_status = QLabel("STOPPED")
        pos_layout.addWidget(self.lbl_left_pos, 1, 1)
        pos_layout.addWidget(self.lbl_left_delta, 1, 2)
        pos_layout.addWidget(self.lbl_left_status, 1, 3)

        # Right Puller row
        pos_layout.addWidget(QLabel("Right Puller:"), 2, 0)
        self.lbl_right_pos = QLabel("0.000 mm")
        self.lbl_right_pos.setFont(mono_font)
        self.lbl_right_delta = QLabel("+0.000 mm")
        self.lbl_right_delta.setFont(mono_font)
        self.lbl_right_status = QLabel("STOPPED")
        pos_layout.addWidget(self.lbl_right_pos, 2, 1)
        pos_layout.addWidget(self.lbl_right_delta, 2, 2)
        pos_layout.addWidget(self.lbl_right_status, 2, 3)

        # Net Slack / Separation Change
        pos_layout.addWidget(QLabel("Total Slack Added:"), 3, 0)
        self.lbl_total_slack = QLabel("+0.000 mm")
        self.lbl_total_slack.setFont(mono_bold)
        self.lbl_total_slack.setStyleSheet("color: #0284c7;")
        pos_layout.addWidget(self.lbl_total_slack, 3, 1)
        self.lbl_pull_slack = QLabel("vs. straight: ---")
        self.lbl_pull_slack.setFont(mono_font)
        if self.slack_ref is not None:
            pe = self.slack_ref["pull_end"]
            tension = self.slack_ref["tension"]
            self.lbl_pull_slack.setToolTip(
                "Slack added since the taper was straight: the end of the pull, or after tensioning it (Tension, "
                "in the main window, before opening the wizard). Saved with every spectrum, and does not restart "
                "when the wizard is opened again.\n"
                f"Puller positions at the end of the pull: {pe[0]:.3f} mm, {pe[1]:.3f} mm\n"
                + (f"Tensioned after the pull by {tension:.3f} mm, to: {self.pull_end_pos[0]:.3f} mm, "
                   f"{self.pull_end_pos[1]:.3f} mm" if abs(tension) >= 5e-4 else "Not tensioned after the pull"))
        else:
            self.lbl_pull_slack.setToolTip("No pull since the program started: the slack can only be measured from "
                                           "where the motors were when the wizard was opened.")
        pos_layout.addWidget(self.lbl_pull_slack, 3, 2, 1, 2)

        # Live Detector / Optical Power Reading
        pos_layout.addWidget(QLabel("Detector / Power:"), 4, 0)
        self.lbl_power_mw = QLabel("--- mW")
        self.lbl_power_mw.setFont(mono_bold)
        self.lbl_power_mw.setStyleSheet("color: #16a34a;")
        self.lbl_power_db = QLabel("--- dBm (Loss: --- dB)")
        self.lbl_power_db.setFont(mono_font)
        self.lbl_detector_v = QLabel("--- V")
        self.lbl_detector_v.setFont(mono_font)
        pos_layout.addWidget(self.lbl_power_mw, 4, 1)
        pos_layout.addWidget(self.lbl_power_db, 4, 2)
        pos_layout.addWidget(self.lbl_detector_v, 4, 3)

        # Lock column minimum widths with extra margin so fluctuating values never shift the layout
        pos_layout.setColumnMinimumWidth(1, fm.horizontalAdvance("-999.9999 mW") + 16)
        pos_layout.setColumnMinimumWidth(2, fm.horizontalAdvance("-100.00 dBm (Loss: -100.00 dB)") + 16)
        pos_layout.setColumnMinimumWidth(3, fm.horizontalAdvance("-10.000 V") + 16)

        main_layout.addWidget(pos_group)

        # --- 3. Step Jog Controls ---
        step_group = QGroupBox("Micro-Step Jogging", self)
        step_layout = QVBoxLayout(step_group)

        # Step size configuration
        step_config_layout = QHBoxLayout()
        step_config_layout.addWidget(QLabel("Step Size:"))
        self.spin_step_size = QDoubleSpinBox(self)
        self.spin_step_size.setRange(0.001, 10.0)
        self.spin_step_size.setDecimals(3)
        self.spin_step_size.setSingleStep(0.01)
        self.spin_step_size.setValue(0.100)
        self.spin_step_size.setSuffix(" mm")
        self.spin_step_size.setMinimumWidth(130)
        step_config_layout.addWidget(self.spin_step_size)

        # Quick preset buttons
        step_config_layout.addWidget(QLabel("Presets:"))
        for preset_val, preset_label in [(0.01, "10µm"), (0.05, "50µm"), (0.1, "0.1mm"), (0.5, "0.5mm"), (1.0, "1mm")]:
            p_btn = QPushButton(preset_label, self)
            p_btn.setMinimumWidth(56)
            p_btn.setStyleSheet(preset_btn_stylesheet)
            p_btn.clicked.connect(lambda _, v=preset_val: self.spin_step_size.setValue(v))
            step_config_layout.addWidget(p_btn)
        step_config_layout.addStretch()
        step_layout.addLayout(step_config_layout)

        # Step Jog Buttons
        jog_btn_layout = QHBoxLayout()
        self.btn_step_tighten = QPushButton("◀◀ Step BW (- Tighten)", self)
        self.btn_step_tighten.setStyleSheet("""
        QPushButton {
            background-color: #d97706;
            color: #ffffff;
            font-weight: bold;
            padding: 9px;
            border: 1px solid #b45309;
            border-radius: 5px;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #b45309;
        }
        QPushButton:pressed {
            background-color: #92400e;
        }
        """)
        self.btn_step_tighten.clicked.connect(self.do_step_tighten)

        self.btn_step_loosen = QPushButton("Step FW (+ Loosen) ▶▶", self)
        self.btn_step_loosen.setStyleSheet("""
        QPushButton {
            background-color: #0284c7;
            color: #ffffff;
            font-weight: bold;
            padding: 9px;
            border: 1px solid #0369a1;
            border-radius: 5px;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #0369a1;
        }
        QPushButton:pressed {
            background-color: #075985;
        }
        """)
        self.btn_step_loosen.clicked.connect(self.do_step_loosen)

        jog_btn_layout.addWidget(self.btn_step_tighten)
        jog_btn_layout.addWidget(self.btn_step_loosen)
        step_layout.addLayout(jog_btn_layout)

        main_layout.addWidget(step_group)

        # --- 4. Continuous Slow Creep Controls (Press & Hold) ---
        creep_group = QGroupBox("Continuous Slow Creep (Press & Hold)", self)
        creep_layout = QVBoxLayout(creep_group)

        creep_config_layout = QHBoxLayout()
        creep_config_layout.addWidget(QLabel("Creep Velocity:"))
        self.spin_creep_vel = QDoubleSpinBox(self)
        self.spin_creep_vel.setRange(0.001, 2.0)
        self.spin_creep_vel.setDecimals(3)
        self.spin_creep_vel.setSingleStep(0.005)
        self.spin_creep_vel.setValue(0.020)
        self.spin_creep_vel.setSuffix(" mm/s")
        self.spin_creep_vel.setMinimumWidth(130)
        creep_config_layout.addWidget(self.spin_creep_vel)

        for c_val, c_lbl in [(0.01, "0.01mm/s"), (0.02, "0.02mm/s"), (0.05, "0.05mm/s"), (0.1, "0.1mm/s")]:
            c_btn = QPushButton(c_lbl, self)
            c_btn.setMinimumWidth(75)
            c_btn.setStyleSheet(preset_btn_stylesheet)
            c_btn.clicked.connect(lambda _, v=c_val: self.spin_creep_vel.setValue(v))
            creep_config_layout.addWidget(c_btn)
        creep_config_layout.addStretch()
        creep_layout.addLayout(creep_config_layout)

        # Press & Hold buttons
        hold_btn_layout = QHBoxLayout()
        self.btn_hold_tighten = QPushButton("▲ HOLD to Tighten (BW -)", self)
        self.btn_hold_tighten.setStyleSheet("""
        QPushButton {
            background-color: #ea580c;
            color: #ffffff;
            font-weight: bold;
            padding: 11px;
            border: 1px solid #c2410c;
            border-radius: 5px;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #c2410c;
        }
        QPushButton:pressed {
            background-color: #9a3412;
        }
        """)
        self.btn_hold_tighten.pressed.connect(self.on_hold_tighten_pressed)
        self.btn_hold_tighten.released.connect(self.on_hold_released)

        self.btn_hold_loosen = QPushButton("▼ HOLD to Loosen (FW +)", self)
        self.btn_hold_loosen.setStyleSheet("""
        QPushButton {
            background-color: #2563eb;
            color: #ffffff;
            font-weight: bold;
            padding: 11px;
            border: 1px solid #1d4ed8;
            border-radius: 5px;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #1d4ed8;
        }
        QPushButton:pressed {
            background-color: #1e40af;
        }
        """)
        self.btn_hold_loosen.pressed.connect(self.on_hold_loosen_pressed)
        self.btn_hold_loosen.released.connect(self.on_hold_released)

        hold_btn_layout.addWidget(self.btn_hold_tighten)
        hold_btn_layout.addWidget(self.btn_hold_loosen)
        creep_layout.addLayout(hold_btn_layout)

        main_layout.addWidget(creep_group)

        # --- 5. Safety & Actions Bottom Bar ---
        action_layout = QHBoxLayout()

        self.btn_reset_pos = QPushButton("Reset to Start Pos", self)
        self.btn_reset_pos.setToolTip("Return both motors to the position when the wizard opened")
        self.btn_reset_pos.setStyleSheet("""
        QPushButton {
            background-color: #475569;
            color: #ffffff;
            padding: 7px 12px;
            border: 1px solid #334155;
            border-radius: 4px;
            font-weight: 500;
        }
        QPushButton:hover {
            background-color: #334155;
        }
        """)
        self.btn_reset_pos.clicked.connect(self.do_reset_positions)

        self.btn_stop = QPushButton("STOP", self)
        self.btn_stop.setStyleSheet("""
        QPushButton {
            background-color: #dc2626;
            color: #ffffff;
            font-weight: bold;
            padding: 7px 18px;
            border: 1px solid #b91c1c;
            border-radius: 4px;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #b91c1c;
        }
        QPushButton:pressed {
            background-color: #991b1b;
        }
        """)
        self.btn_stop.clicked.connect(self.do_emergency_stop)

        self.btn_done = QPushButton("Done / Close", self)
        self.btn_done.setStyleSheet("""
        QPushButton {
            background-color: #16a34a;
            color: #ffffff;
            font-weight: bold;
            padding: 7px 18px;
            border: 1px solid #15803d;
            border-radius: 4px;
            font-size: 13px;
        }
        QPushButton:hover {
            background-color: #15803d;
        }
        """)
        self.btn_done.clicked.connect(self.accept)

        action_layout.addWidget(self.btn_reset_pos)
        action_layout.addWidget(self.btn_stop)
        action_layout.addStretch()
        action_layout.addWidget(self.btn_done)

        main_layout.addLayout(action_layout)

        # Update initial step instructions
        self.update_step_guide()

    # --- Spectrum panel ---
    def init_spectrum_panel(self):
        group = QGroupBox("Transmission Spectrum", self)
        layout = QVBoxLayout(group)
        
        # Sweep settings. These are the same as in the 'Laser' tab of the main window
        m = self.main
        settings_layout = QGridLayout()
        def mirror_spin(source, label, row, col):
            spin = QDoubleSpinBox(self)
            spin.setDecimals(source.decimals())
            spin.setRange(source.minimum(), source.maximum())
            spin.setSingleStep(source.singleStep())
            spin.setSuffix(source.suffix())
            spin.setValue(source.value())
            spin.setKeyboardTracking(False)
            spin.setToolTip(source.toolTip())
            spin.setMinimumWidth(max(120, spin.sizeHint().width()))  # The whole value must be visible
            spin.valueChanged.connect(source.setValue)
            lbl = QLabel(label)
            settings_layout.addWidget(lbl, row, col)
            settings_layout.addWidget(spin, row, col + 1)
            return spin, lbl
        
        self.spin_sweep_start, _ = mirror_spin(m.sweepStartSpin, "From:", 0, 0)
        self.spin_sweep_stop, _ = mirror_spin(m.sweepStopSpin, "To:", 0, 2)
        
        self.combo_sweep_speed = QComboBox(self)
        self.combo_sweep_speed.addItems([m.sweepSpeedCombo.itemText(i) for i in range(m.sweepSpeedCombo.count())])
        self.combo_sweep_speed.setCurrentIndex(m.sweepSpeedCombo.currentIndex())
        self.combo_sweep_speed.currentIndexChanged.connect(m.sweepSpeedCombo.setCurrentIndex)
        settings_layout.addWidget(QLabel("Speed:"), 0, 4)
        settings_layout.addWidget(self.combo_sweep_speed, 0, 5)
        layout.addLayout(settings_layout)
        
        # Note saved with the spectrum
        note_layout = QHBoxLayout()
        note_layout.addWidget(QLabel("Note:"))
        self.edit_sweep_note = QLineEdit(self)
        self.edit_sweep_note.setPlaceholderText("Saved with the next spectrum (e.g. polarization controller setting)")
        note_layout.addWidget(self.edit_sweep_note)
        layout.addLayout(note_layout)
        
        # Sweep control
        sweep_layout = QHBoxLayout()
        self.btn_sweep = QPushButton("Sweep", self)
        self.btn_sweep.setStyleSheet("""
        QPushButton {
            background-color: #7c3aed; color: #ffffff; font-weight: bold;
            padding: 9px 22px; border: 1px solid #6d28d9; border-radius: 5px; font-size: 13px;
        }
        QPushButton:hover { background-color: #6d28d9; }
        QPushButton:disabled { background-color: #c4b5fd; border-color: #c4b5fd; }
        """)
        self.btn_sweep.clicked.connect(self.start_sweep)
        self.btn_sweep_abort = QPushButton("Abort", self)
        self.btn_sweep_abort.setEnabled(False)
        self.btn_sweep_abort.clicked.connect(self.abort_sweep)
        self.sweep_progress = QProgressBar(self)
        self.sweep_progress.setRange(0, 1000)
        sweep_layout.addWidget(self.btn_sweep)
        sweep_layout.addWidget(self.btn_sweep_abort)
        sweep_layout.addWidget(self.sweep_progress, 1)
        layout.addLayout(sweep_layout)
        
        self.lbl_sweep_status = QLabel("No spectrum measured yet.", self)
        self.lbl_sweep_status.setWordWrap(True)
        layout.addWidget(self.lbl_sweep_status)
        
        # Graphs: last spectrum, and all spectra of this session
        self.figure = Figure(constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.ax_spec, self.ax_map = self.figure.subplots(2, 1, sharex=True)
        self.line_prev, = self.ax_spec.plot([], [], color="#cbd5e1", lw=1.0, label="Previous")
        self.line_spec, = self.ax_spec.plot([], [], color="#7c3aed", lw=1.2, label="Last")
        self.ax_spec.set_ylabel("Transm. Power (mW)")
        self.ax_spec.grid(True)
        self.ax_spec.legend(loc="lower right")
        self.img_map = self.ax_map.imshow(np.zeros((1, 2)), aspect="auto", origin="lower", cmap="viridis",
                                          extent=(0.0, 1.0, 0.5, 1.5), interpolation="nearest")
        self.ax_map.set_xlabel("Wavelength (nm)")
        self.ax_map.set_ylabel("Slack added (mm), per spectrum")
        self.ax_map.set_yticks([])
        layout.addWidget(self.canvas, 1)
        
        return group
    
    # --- Camera panel ---
    def init_camera_panel(self):
        group = QGroupBox("Camera", self)
        layout = QVBoxLayout(group)
        m = self.main
        
        # Camera selection. Same settings as in the 'Camera' tab of the main window
        select_layout = QHBoxLayout()
        select_layout.addWidget(QLabel("Camera number:"))
        self.spin_camera = QSpinBox(self)
        self.spin_camera.setRange(m.cameraIndexSpin.minimum(), m.cameraIndexSpin.maximum())
        self.spin_camera.setValue(m.cameraIndexSpin.value())
        self.spin_camera.setToolTip(m.cameraIndexSpin.toolTip())
        self.spin_camera.setMinimumWidth(max(70, self.spin_camera.sizeHint().width()))
        self.spin_camera.valueChanged.connect(m.cameraIndexSpin.setValue)
        select_layout.addWidget(self.spin_camera)
        self.check_camera_sim = QCheckBox("Simulate", self)
        self.check_camera_sim.setChecked(m.cameraSimCheck.isChecked())
        self.check_camera_sim.clicked.connect(m.cameraSimCheck.setChecked)
        select_layout.addWidget(self.check_camera_sim)
        self.btn_camera = QPushButton("Turn On", self)
        self.btn_camera.setCheckable(True)
        self.btn_camera.clicked.connect(self.toggle_camera)
        select_layout.addWidget(self.btn_camera)
        self.btn_flip_h = QPushButton("Flip H", self)
        self.btn_flip_h.setCheckable(True)
        self.btn_flip_h.setToolTip("Flip the camera image horizontally")
        self.btn_flip_h.setChecked(self.camera.flip_h)
        self.btn_flip_h.toggled.connect(self.toggle_flip_h)
        select_layout.addWidget(self.btn_flip_h)
        self.btn_flip_v = QPushButton("Flip V", self)
        self.btn_flip_v.setCheckable(True)
        self.btn_flip_v.setToolTip("Flip the camera image vertically")
        self.btn_flip_v.setChecked(self.camera.flip_v)
        self.btn_flip_v.toggled.connect(self.toggle_flip_v)
        select_layout.addWidget(self.btn_flip_v)
        select_layout.addStretch()
        layout.addLayout(select_layout)
        
        # Image
        self.lbl_camera = QLabel("Camera off", self)
        self.lbl_camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_camera.setMinimumSize(240, 180)
        self.lbl_camera.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.lbl_camera.setStyleSheet("background-color: #0f172a; color: #94a3b8; border-radius: 4px;")
        layout.addWidget(self.lbl_camera, 1)
        
        # Saving
        save_layout = QHBoxLayout()
        self.btn_snapshot = QPushButton("Save Snapshot", self)
        self.btn_snapshot.clicked.connect(self.save_snapshot)
        self.btn_record = QPushButton("Record Video", self)
        self.btn_record.setCheckable(True)
        self.btn_record.clicked.connect(self.toggle_video)
        self.check_auto_snapshot = QCheckBox("Snapshot with each spectrum", self)
        self.check_auto_snapshot.setChecked(True)
        save_layout.addWidget(self.btn_snapshot)
        save_layout.addWidget(self.btn_record)
        save_layout.addWidget(self.check_auto_snapshot)
        save_layout.addStretch()
        layout.addLayout(save_layout)
        
        self.lbl_camera_status = QLabel("", self)
        self.lbl_camera_status.setWordWrap(True)
        layout.addWidget(self.lbl_camera_status)
        
        self.update_camera_controls()
        return group
    
    def update_camera_controls(self):
        on = self.camera.ok
        self.btn_camera.setChecked(on)
        self.btn_camera.setText("Turn Off" if on else "Turn On")
        self.spin_camera.setEnabled(not on)
        self.check_camera_sim.setEnabled(not on)
        self.btn_snapshot.setEnabled(on)
        self.btn_record.setEnabled(on)
        self.btn_record.setChecked(self.camera.recording)
        self.btn_record.setText("Stop Recording" if self.camera.recording else "Record Video")

    def toggle_flip_h(self, checked: bool):
        self.camera.set_flip_h(checked)
        if hasattr(self.main, "cameraFlipHCheck"):
            self.main.cameraFlipHCheck.setChecked(checked)
        self.update_camera()

    def toggle_flip_v(self, checked: bool):
        self.camera.set_flip_v(checked)
        if hasattr(self.main, "cameraFlipVCheck"):
            self.main.cameraFlipVCheck.setChecked(checked)
        self.update_camera()
    
    def toggle_camera(self):
        if self.btn_camera.isChecked():
            if self.camera.open(self.spin_camera.value(), self.check_camera_sim.isChecked()):
                self.camera_frame_count = -1
                self.camera_timer.start()
                self.lbl_camera_status.setText("")
            else:
                self.lbl_camera_status.setText(f"Could not turn the camera on: {self.camera.last_error}")
        else:
            self.camera_timer.stop()
            self.camera.close()
            self.lbl_camera.setPixmap(QPixmap())
            self.lbl_camera.setText("Camera off")
        self.update_camera_controls()
        
    def update_camera(self):
        # Only redraw when there is a new image
        if self.camera.frame_count == self.camera_frame_count:
            return
        frame = self.camera.get_frame()
        if frame is None:
            return
        self.camera_frame_count = self.camera.frame_count
        
        h, w = frame.shape[:2]
        image = QImage(frame.data, w, h, frame.strides[0], QImage.Format.Format_BGR888)
        pixmap = QPixmap.fromImage(image).scaled(self.lbl_camera.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                 Qt.TransformationMode.SmoothTransformation)
        self.lbl_camera.setPixmap(pixmap)
        
    def get_spectra_folder(self):
        """Folder where everything from this session is saved. Created when first needed."""
        if self.spectra_folder == "":
            self.spectra_folder = self.main.new_spectra_folder("loop")
        return self.spectra_folder
        
    def save_snapshot(self):
        try:
            name = f"snapshot_{datetime.now().strftime('%H_%M_%S')}.png"
            if self.camera.save_snapshot(os.path.join(self.get_spectra_folder(), name)):
                self.lbl_camera_status.setText(f"Saved: {os.path.basename(self.spectra_folder)}/{name}")
            else:
                self.lbl_camera_status.setText("Snapshot NOT saved: no image from the camera.")
        except Exception as e:
            self.lbl_camera_status.setText(f"Snapshot NOT saved: {e}")
            
    def toggle_video(self):
        if self.btn_record.isChecked():
            try:
                name = f"video_{datetime.now().strftime('%H_%M_%S')}.avi"
                if self.camera.start_recording(os.path.join(self.get_spectra_folder(), name)):
                    self.lbl_camera_status.setText(f"Recording: {os.path.basename(self.spectra_folder)}/{name}")
                else:
                    self.lbl_camera_status.setText("Could not start recording the video.")
            except Exception as e:
                self.lbl_camera_status.setText(f"Could not start recording the video: {e}")
        else:
            self.camera.stop_recording()
            self.lbl_camera_status.setText("Video saved.")
        self.update_camera_controls()
        
    def close_camera(self):
        if self.camera is not None:
            self.camera_timer.stop()
            self.camera.close()

    def get_slack(self):
        """Slack added since the wizard was opened (both motors), in mm."""
        if not self.left_motor or not self.right_motor:
            return 0.0
        return (self.left_motor.get_position() - self.init_left_pos) + \
               (self.right_motor.get_position() - self.init_right_pos)

    def get_slack_from_straight(self) -> float|None:
        """Slack added since the taper was straight (both motors), in mm. None if unknown."""
        if self.pull_end_pos is None or not self.left_motor or not self.right_motor:
            return None
        return (self.left_motor.get_position() - self.pull_end_pos[0]) + \
               (self.right_motor.get_position() - self.pull_end_pos[1])

    def start_sweep(self):
        if self.left_motor and self.right_motor and (self.left_motor.motor_moving() or self.right_motor.motor_moving()):
            QMessageBox.warning(self, "Sweep", "Wait until the motors stop before measuring a spectrum.")
            return
        
        error = self.main.prepare_sweep()
        if error == "":
            l_pos = self.left_motor.get_position() if self.left_motor else 0.0
            r_pos = self.right_motor.get_position() if self.right_motor else 0.0
            # Image of the loop, as it is right before the sweep
            index = len(self.spectra) + 1
            snapshot = ""
            if self.camera is not None and self.camera.ok and self.check_auto_snapshot.isChecked():
                try:
                    snapshot = f"spectrum_{index:03d}.png"
                    if not self.camera.save_snapshot(os.path.join(self.get_spectra_folder(), snapshot)):
                        snapshot = ""
                except Exception as e:
                    print(f"Error saving snapshot: {e}")
                    snapshot = ""
            
            metadata = {
                "index": index,
                "snapshot": snapshot,
                "note": self.edit_sweep_note.text().strip(),
                "left_pos": l_pos,
                "right_pos": r_pos,
                "left_displacement": l_pos - self.init_left_pos,
                "right_displacement": r_pos - self.init_right_pos,
                "slack": self.get_slack(),
                "slack_from_straight": self.get_slack_from_straight(),
                "straight_left_pos": self.pull_end_pos[0] if self.slack_ref is not None else None,
                "straight_right_pos": self.pull_end_pos[1] if self.slack_ref is not None else None,
                "pull_end_left_pos": self.slack_ref["pull_end"][0] if self.slack_ref is not None else None,
                "pull_end_right_pos": self.slack_ref["pull_end"][1] if self.slack_ref is not None else None,
                "tension_after_pull": self.slack_ref["tension"] if self.slack_ref is not None else None,
            }
            error = self.spectrum.start(metadata)
        if error != "":
            QMessageBox.warning(self, "Sweep", error)
            return
        
        # The loop must not change during the sweep
        self.motors_panel.setEnabled(False)
        self.btn_sweep.setEnabled(False)
        self.btn_sweep_abort.setEnabled(True)
        self.sweep_progress.setValue(0)
        self.lbl_sweep_status.setText("Sweeping...")
        self.sweep_timer.start()
        
    def abort_sweep(self):
        self.spectrum.abort()
        
    def update_sweep(self):
        running = self.spectrum.update()
        self.sweep_progress.setValue(int(1000*self.spectrum.progress))
        if running:
            return
        
        self.sweep_timer.stop()
        self.motors_panel.setEnabled(True)
        self.btn_sweep.setEnabled(True)
        self.btn_sweep_abort.setEnabled(False)
        
        result = self.spectrum.get_result()
        if result is None:
            self.sweep_progress.setValue(0)
            self.lbl_sweep_status.setText(self.spectrum.error)
            return
        
        # Save
        index = result["info"]["metadata"]["index"]
        slack = result["info"]["metadata"]["slack"]
        info = result["info"]
        pull_slack = info["metadata"].get("slack_from_straight")
        details = f"{slack:+.3f} mm slack" + (f" ({pull_slack:+.3f} mm vs. straight taper)" if pull_slack is not None else "")
        if info.get("laser_output_was_off"):
            details += ", laser output was off and was turned back on"
        details += ", started {info['start_delay']:.2f} s after the command, wavelengths from "
        if info["wavelength_source"] == "triggers":
            details += f"{info['trigger_count']} triggers ({info['speed_measured']:.3f} nm/s)"
        elif info["wavelength_source"] == "trigger_start":
            details += "the start trigger"
        else:
            details += "the time since the start (no triggers)"
        try:
            filename = os.path.join(self.get_spectra_folder(), f"spectrum_{index:03d}.npz")
            self.spectrum.save(result, filename)
            self.lbl_sweep_status.setText(f"Spectrum {index} ({details}) saved: "
                                          f"{os.path.basename(self.spectra_folder)}/{os.path.basename(filename)}")
            self.lbl_sweep_status.setToolTip(filename)
        except Exception as e:
            self.lbl_sweep_status.setText(f"Spectrum {index} measured, but NOT saved: {e}")
        
        self.spectra.append((result["wavelength"], result["power"], slack))
        self.edit_sweep_note.clear()
        self.update_spectrum_graphs()
        
    def update_spectrum_graphs(self):
        if len(self.spectra) == 0:
            return
        
        # Last and previous spectra
        wl, power, _ = self.spectra[-1]
        self.line_spec.set_data(wl, power)
        if len(self.spectra) > 1:
            self.line_prev.set_data(self.spectra[-2][0], self.spectra[-2][1])
        self.ax_spec.relim()
        self.ax_spec.autoscale(True)
        
        # All spectra, one row each (in the order they were measured)
        wl_min = min(s[0][0] for s in self.spectra if len(s[0]) > 0)
        wl_max = max(s[0][-1] for s in self.spectra if len(s[0]) > 0)
        wl_grid = np.linspace(wl_min, wl_max, 1000)
        data = np.array([np.interp(wl_grid, s[0], s[1], left=np.nan, right=np.nan) for s in self.spectra])
        n = len(self.spectra)
        self.img_map.set_data(data)
        self.img_map.set_extent((wl_min, wl_max, 0.5, n + 0.5))
        self.img_map.set_clim(np.nanmin(data), np.nanmax(data))
        ticks = np.unique(np.linspace(1, n, min(n, 10)).astype(int))
        self.ax_map.set_yticks(ticks)
        self.ax_map.set_yticklabels([f"{self.spectra[i - 1][2]:+.3f}" for i in ticks])
        self.ax_spec.set_xlim(wl_min, wl_max)
        self.canvas.draw_idle()

    # --- Step Guide Updates ---
    def set_step(self, step_id):
        self.current_step = step_id
        self.update_step_guide()

    def update_step_guide(self):
        instructions = [
            "<b>Step 1: Loosen the Fiber (FW / Positive Motion)</b><br>"
            "Move both motors forward (+) to create sufficient slack. "
            "Use small steps or press-and-hold creep so the microfiber does not sag excessively onto fixtures.",

            "<b>Step 2: Form the Loop / Twist</b><br>"
            "Rotate the fiber holder/clamp or use a clean tungsten micro-probe to overlap the microfiber waist onto itself. "
            "Van der Waals attraction will hold the contact junction once aligned.",

            "<b>Step 3: Tension & Tune (BW / Negative Motion)</b><br>"
            "Move both motors backward (-) slowly in micro-steps (10-50 µm) to tighten the loop. "
            "Monitor optical transmission / resonance spectra until target diameter and coupling are reached."
        ]
        if 0 <= self.current_step < len(instructions):
            self.guide_text.setHtml(instructions[self.current_step])

    # --- Real-time Telemetry Updates ---
    def update_telemetry(self):
        if self.main is not None and getattr(self.main, "data", None) is not None:
            data = self.main.data
            volt = float(data.monitor_val)
            power_mw = max(0.0, float(data.get_monitor_power(db=False)))
            power_dbm = max(float(data.get_monitor_power(db=True)), -100.0)
            ref_dbm = self.main.refpowIndSpin.value() if hasattr(self.main, "refpowIndSpin") else 0.0
            loss_db = ref_dbm - power_dbm

            # Deadband around zero to prevent sign flickering from ADC noise
            if volt > -0.005:
                volt = max(0.0, volt)
            power_dbm = 0.0 if abs(power_dbm) < 0.01 else power_dbm
            loss_db = 0.0 if abs(loss_db) < 0.01 else loss_db

            dbm_str = f"{power_dbm:+.2f} dBm"
            self.lbl_power_mw.setText(f"{power_mw:.4f} mW" if power_mw < 1.0 else f"{power_mw:.3f} mW")
            self.lbl_power_db.setText(f"{dbm_str:<11s} (Loss: {loss_db:+.2f} dB)")
            self.lbl_power_db.setToolTip(f"Reference power: {ref_dbm:.2f} dBm")
            self.lbl_detector_v.setText(f"{volt:.3f} V")

        if not self.left_motor or not self.right_motor:
            return

        l_pos = max(0.0, self.left_motor.get_position())
        r_pos = max(0.0, self.right_motor.get_position())

        dl = l_pos - self.init_left_pos
        dr = r_pos - self.init_right_pos
        total_slack = dl + dr

        dl = 0.0 if abs(dl) < 5e-4 else dl
        dr = 0.0 if abs(dr) < 5e-4 else dr
        total_slack = 0.0 if abs(total_slack) < 5e-4 else total_slack

        self.lbl_left_pos.setText(f"{l_pos:.3f} mm")
        self.lbl_right_pos.setText(f"{r_pos:.3f} mm")

        self.lbl_left_delta.setText(f"{dl:+.3f} mm")
        self.lbl_right_delta.setText(f"{dr:+.3f} mm")

        self.lbl_total_slack.setText(f"{total_slack:+.3f} mm")
        if self.pull_end_pos is not None:
            pull_slack = (l_pos - self.pull_end_pos[0]) + (r_pos - self.pull_end_pos[1])
            pull_slack = 0.0 if abs(pull_slack) < 5e-4 else pull_slack
            self.lbl_pull_slack.setText(f"vs. straight: {pull_slack:+.3f} mm")

        # Update motion indicators
        if self.left_motor.motor_moving():
            self.lbl_left_status.setText("MOVING")
            self.lbl_left_status.setStyleSheet("color: #0284c7; font-weight: bold;")
        else:
            self.lbl_left_status.setText("STOPPED")
            self.lbl_left_status.setStyleSheet("color: #16a34a; font-weight: 500;")

        if self.right_motor.motor_moving():
            self.lbl_right_status.setText("MOVING")
            self.lbl_right_status.setStyleSheet("color: #0284c7; font-weight: bold;")
        else:
            self.lbl_right_status.setText("STOPPED")
            self.lbl_right_status.setStyleSheet("color: #16a34a; font-weight: 500;")

    # --- Step Jog Execution ---
    def do_step_loosen(self):
        """Move both motors in positive direction (+ step) to loosen fiber."""
        step = self.spin_step_size.value()
        self._execute_step(+step, +step)

    def do_step_tighten(self):
        """Move both motors in negative direction (- step) to tighten fiber."""
        step = self.spin_step_size.value()
        self._execute_step(-step, -step)

    def _execute_step(self, d_left, d_right):
        if not self.left_motor or not self.right_motor:
            return

        l_pos = self.left_motor.get_position()
        r_pos = self.right_motor.get_position()

        target_l = l_pos + d_left
        target_r = r_pos + d_right

        # Physical travel range checks [0.0, max_pos]
        max_pos_l = getattr(self.left_motor, "max_pos", 100.0)
        min_pos_l = getattr(self.left_motor, "min_pos", 0.0)
        max_pos_r = getattr(self.right_motor, "max_pos", 100.0)
        min_pos_r = getattr(self.right_motor, "min_pos", 0.0)

        if not (min_pos_l <= target_l <= max_pos_l):
            QMessageBox.warning(
                self, "Travel Limit",
                f"Left puller target {target_l:.3f} mm is outside physical limits [{min_pos_l:.1f}, {max_pos_l:.1f}] mm."
            )
            return
        if not (min_pos_r <= target_r <= max_pos_r):
            QMessageBox.warning(
                self, "Travel Limit",
                f"Right puller target {target_r:.3f} mm is outside physical limits [{min_pos_r:.1f}, {max_pos_r:.1f}] mm."
            )
            return

        self.left_motor.go_to(target_l)
        self.right_motor.go_to(target_r)

    # --- Continuous Creep (Press & Hold) ---
    def on_hold_loosen_pressed(self):
        """Start moving both motors slowly in positive direction while held."""
        self._start_continuous_move(direction=+1)

    def on_hold_tighten_pressed(self):
        """Start moving both motors slowly in negative direction while held."""
        self._start_continuous_move(direction=-1)

    def _start_continuous_move(self, direction):
        if not self.left_motor or not self.right_motor:
            return

        self.is_holding_continuous = True
        creep_vel = self.spin_creep_vel.value()

        # Save default velocities
        self.default_left_vel = self.left_motor.vel
        self.default_right_vel = self.right_motor.vel

        # Set slow creep velocity
        self.left_motor.set_velocity(creep_vel)
        self.right_motor.set_velocity(creep_vel)

        # Start continuous motion
        if direction > 0:
            self.left_motor.move(self.left_motor.MoveDirection.POSITIVE)
            self.right_motor.move(self.right_motor.MoveDirection.POSITIVE)
        else:
            self.left_motor.move(self.left_motor.MoveDirection.NEGATIVE)
            self.right_motor.move(self.right_motor.MoveDirection.NEGATIVE)

    def on_hold_released(self):
        """Stop motors immediately on button release and restore default velocities."""
        if not self.left_motor or not self.right_motor:
            return

        self.left_motor.stop(0)
        self.right_motor.stop(0)

        # Restore default velocities
        self.left_motor.set_velocity(self.default_left_vel)
        self.right_motor.set_velocity(self.default_right_vel)
        self.is_holding_continuous = False

    # --- Emergency Stop & Reset ---
    def do_emergency_stop(self):
        """Stop all motor motion immediately."""
        if self.left_motor:
            self.left_motor.stop(0)
        if self.right_motor:
            self.right_motor.stop(0)
        self.is_holding_continuous = False

    def do_reset_positions(self):
        """Return both puller motors to the initial positions recorded when the wizard was opened."""
        reply = QMessageBox.question(
            self, "Reset Positions",
            f"Return both motors to initial positions?\nLeft: {self.init_left_pos:.3f} mm\nRight: {self.init_right_pos:.3f} mm",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.do_emergency_stop()
            if self.left_motor:
                self.left_motor.go_to(self.init_left_pos)
            if self.right_motor:
                self.right_motor.go_to(self.init_right_pos)

    def done(self, result):
        """Closing with the 'Done' button: make sure nothing is left running."""
        if self.spectrum is not None and self.spectrum.running:
            self.spectrum.abort()
        self.sweep_timer.stop()
        self.close_camera()
        super().done(result)

    def closeEvent(self, event):
        """Ensure all motors are stopped, velocities restored, and timer killed upon closing."""
        self.do_emergency_stop()
        if self.left_motor:
            self.left_motor.set_velocity(self.default_left_vel)
        if self.right_motor:
            self.right_motor.set_velocity(self.default_right_vel)
        if self.spectrum is not None and self.spectrum.running:
            self.spectrum.abort()
        self.sweep_timer.stop()
        self.close_camera()
        self.telemetry_timer.stop()
        event.accept()
