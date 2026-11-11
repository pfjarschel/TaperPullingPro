# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
Easy mode panel: a simplified view of the taper pulling program, with only
what is needed to fabricate a taper from start to finish, in the order it is
used. Everything here is a mirror of a control of the complete (Advanced)
view: both always show the same values, and all the other settings keep
whatever was set in the Advanced view.
"""

import os
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLabel,
    QPushButton, QDoubleSpinBox, QCheckBox, QLineEdit, QProgressBar
)
from PyQt6.QtGui import QFont


class EasyModePanel(QWidget):
    """
    Simplified view of the main window. Needs the main window, where
    everything actually happens.
    """

    status_colors = {"ok": "#16a34a", "busy": "#ca8a04", "off": "#64748b", "error": "#dc2626"}

    def __init__(self, main):
        super().__init__(main)
        self.main = main
        self.mirrors = []  # (widget here, widget in the Advanced view): kept enabled/disabled together

        self.init_ui()

        # Status update timer
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(200)

    # --- Mirrors of the Advanced view controls ---
    def mirror_spin(self, source: QDoubleSpinBox, suffix: str="") -> QDoubleSpinBox:
        spin = QDoubleSpinBox(self)
        spin.setDecimals(source.decimals())
        spin.setRange(source.minimum(), source.maximum())
        spin.setSingleStep(source.singleStep())
        spin.setSuffix(suffix if suffix else source.suffix())
        spin.setValue(source.value())
        spin.setKeyboardTracking(False)
        spin.setToolTip(source.toolTip())
        spin.valueChanged.connect(source.setValue)

        def from_source(value):
            spin.blockSignals(True)
            spin.setRange(source.minimum(), source.maximum())
            spin.setValue(value)
            spin.blockSignals(False)
        source.valueChanged.connect(from_source)

        self.mirrors.append((spin, source))
        return spin

    def mirror_check(self, source: QCheckBox, text: str="") -> QCheckBox:
        check = QCheckBox(text if text else source.text(), self)
        check.setChecked(source.isChecked())
        check.setToolTip(source.toolTip())
        # click(), so that whatever is connected to the original also happens
        check.clicked.connect(lambda checked: source.click() if source.isChecked() != checked else None)
        source.toggled.connect(check.setChecked)
        self.mirrors.append((check, source))
        return check

    def mirror_line(self, source: QLineEdit) -> QLineEdit:
        line = QLineEdit(source.text(), self)
        line.setToolTip(source.toolTip())
        line.textEdited.connect(source.setText)

        def from_source(text):
            if line.text() != text:
                line.setText(text)
        source.textChanged.connect(from_source)

        self.mirrors.append((line, source))
        return line

    def mirror_button(self, source: QPushButton, text: str="", style: str="") -> QPushButton:
        button = QPushButton(text if text else source.text(), self)
        button.setToolTip(source.toolTip())
        button.setCheckable(source.isCheckable())
        if style:
            button.setStyleSheet(style)
        button.setMinimumHeight(30)
        button.clicked.connect(lambda: source.click())
        self.mirrors.append((button, source))
        return button

    def status_label(self) -> QLabel:
        label = QLabel("", self)
        label.setWordWrap(True)
        return label

    def set_status(self, label: QLabel, text: str, kind: str="off"):
        label.setText(text)
        label.setStyleSheet(f"color: {self.status_colors[kind]}; font-weight: bold;")

    # --- UI ---
    def init_ui(self):
        m = self.main
        layout = QHBoxLayout(self)
        layout.setSpacing(12)

        left = QWidget(self)
        left.setFixedWidth(470)
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)
        layout.addWidget(left)

        big_button = "QPushButton { font-weight: bold; }"

        # --- 1. Setup ---
        group = QGroupBox("1. Setup", self)
        grid = QGridLayout(group)

        grid.addWidget(QLabel("Motors:"), 0, 0)
        self.lbl_motors = self.status_label()
        grid.addWidget(self.lbl_motors, 0, 1, 1, 2)
        grid.addWidget(self.mirror_button(m.resetmotorsBut), 0, 3)

        grid.addWidget(QLabel("Laser:"), 1, 0)
        self.lbl_laser = self.status_label()
        grid.addWidget(self.lbl_laser, 1, 1)
        self.btn_laser_connect = self.mirror_button(m.laserConnectBut)
        self.btn_laser_output = self.mirror_button(m.laserOutBut)
        grid.addWidget(self.btn_laser_connect, 1, 2)
        grid.addWidget(self.btn_laser_output, 1, 3)

        grid.addWidget(QLabel("Wavelength:"), 2, 0)
        grid.addWidget(self.mirror_spin(m.laserWlSpin), 2, 1)
        grid.addWidget(QLabel("Power:"), 2, 2)
        grid.addWidget(self.mirror_spin(m.laserPowSpin), 2, 3)

        grid.addWidget(QLabel("Transmission:"), 3, 0)
        self.lbl_power = QLabel("", self)
        grid.addWidget(self.lbl_power, 3, 1, 1, 2)
        btn_ref = self.mirror_button(m.set0powBut, "Set as Reference")
        btn_ref.setToolTip("Takes the power transmitted now (before pulling) as the reference to calculate the loss")
        grid.addWidget(btn_ref, 3, 3)
        grid.setColumnStretch(1, 1)
        left_layout.addWidget(group)

        # --- 2. Taper ---
        group = QGroupBox("2. Taper", self)
        grid = QGridLayout(group)

        grid.addWidget(QLabel("Waist diameter, nominal:"), 0, 0, 1, 2)
        grid.addWidget(self.mirror_spin(m.dwSpin, " µm"), 0, 2, 1, 2)
        grid.addWidget(QLabel("Initial hotzone:"), 1, 0)
        grid.addWidget(self.mirror_spin(m.l0Spin, " mm"), 1, 1)
        alpha_tip = ("How the hotzone changes while pulling: hotzone = initial hotzone + alpha x total pulled.\n"
                     "0: constant hotzone (exponential profile). Positive: the hotzone grows, for a longer waist.\n"
                     "Negative: the hotzone shrinks, for shorter transitions.")
        label = QLabel("Alpha factor:")
        label.setToolTip(alpha_tip)
        grid.addWidget(label, 1, 2)
        spin = self.mirror_spin(m.alphaSpin)
        spin.setToolTip(alpha_tip)
        grid.addWidget(spin, 1, 3)
        grid.addWidget(QLabel("Pulling velocity (each side):"), 2, 0, 1, 2)
        grid.addWidget(self.mirror_spin(m.pullerPullVelSpin, " mm/s"), 2, 2, 1, 2)
        check = self.mirror_check(m.manualStopCheck, "Do not stop automatically (stop it yourself)")
        grid.addWidget(check, 3, 0, 1, 4)
        self.lbl_plan = QLabel("", self)
        self.lbl_plan.setWordWrap(True)
        self.lbl_plan.setStyleSheet("color: #475569;")
        grid.addWidget(self.lbl_plan, 4, 0, 1, 4)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        left_layout.addWidget(group)

        # --- 3. Fabrication ---
        group = QGroupBox("3. Fabrication", self)
        vbox = QVBoxLayout(group)

        self.lbl_stage = QLabel("", self)
        stage_font = QFont()
        stage_font.setPointSize(15)
        stage_font.setBold(True)
        self.lbl_stage.setFont(stage_font)
        self.lbl_stage.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_stage.setMinimumHeight(34)
        vbox.addWidget(self.lbl_stage)

        self.progress = QProgressBar(self)
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        vbox.addWidget(self.progress)
        self.lbl_time = QLabel("", self)
        self.lbl_time.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vbox.addWidget(self.lbl_time)

        row = QHBoxLayout()
        row.addWidget(self.mirror_button(m.gotozeroBut, "Go to Start"))
        row.addWidget(self.mirror_button(m.startBut, "Start Pulling", big_button))
        row.addWidget(self.mirror_button(m.stopBut, "Stop Pulling", big_button))
        row.addWidget(self.mirror_button(m.markBut))
        vbox.addLayout(row)

        grid = QGridLayout()
        value_font = QFont("Monospace")
        value_font.setStyleHint(QFont.StyleHint.TypeWriter)
        value_font.setBold(True)
        self.lbl_pulled = QLabel("", self)
        self.lbl_diameter = QLabel("", self)
        self.lbl_loss = QLabel("", self)
        for i, (text, label) in enumerate((("Total pulled", self.lbl_pulled),
                                           ("Waist diameter, nominal", self.lbl_diameter),
                                           ("Loss", self.lbl_loss))):
            title = QLabel(text, self)
            title.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setFont(value_font)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title.setMinimumHeight(18)
            label.setMinimumHeight(20)
            grid.addWidget(title, 0, i)
            grid.addWidget(label, 1, i)
        vbox.addLayout(grid)
        left_layout.addWidget(group)

        # --- 4. After pulling ---
        group = QGroupBox("4. After pulling", self)
        row = QHBoxLayout(group)
        row.addWidget(self.mirror_button(m.tensionButton))
        row.addWidget(self.mirror_button(m.loopBut, "Loop Wizard", big_button))
        row.addWidget(self.mirror_button(m.cleaveBut))
        left_layout.addWidget(group)

        # --- Data ---
        group = QGroupBox("Data", self)
        grid = QGridLayout(group)
        grid.addWidget(QLabel("Name:"), 0, 0)
        grid.addWidget(self.mirror_line(m.dataPrefixText), 0, 1)
        self.lbl_data = QLabel("", self)
        self.lbl_data.setStyleSheet("color: #475569;")
        grid.addWidget(self.lbl_data, 1, 0, 1, 2)
        left_layout.addWidget(group)

        left_layout.addStretch()

        # --- Bottom: emergency stop and mode switch ---
        row = QHBoxLayout()
        btn_emergency = QPushButton("STOP EVERYTHING", self)
        btn_emergency.setToolTip("Stops all motors immediately and moves the flame away")
        btn_emergency.setStyleSheet("""
        QPushButton {
            background-color: #dc2626; color: #ffffff; font-weight: bold;
            padding: 10px 18px; border: 1px solid #b91c1c; border-radius: 4px;
        }
        QPushButton:hover { background-color: #b91c1c; }
        """)
        btn_emergency.clicked.connect(lambda: m.emerBut.click())
        self.mirrors.append((btn_emergency, m.emerBut))
        row.addWidget(btn_emergency, 1)
        row.addWidget(self.mirror_button(m.modeBut))
        left_layout.addLayout(row)

        # --- Right: graphs (moved here from the Advanced view when this mode is on) ---
        right = QWidget(self)
        self.graphs_layout = QVBoxLayout(right)
        self.graphs_layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(right, 1)

        self.update_status()

    # --- Status ---
    def update_status(self):
        m = self.main
        core = m.core

        # Controls follow the state of the ones they mirror
        for widget, source in self.mirrors:
            if widget.isEnabled() != source.isEnabled():
                widget.setEnabled(source.isEnabled())
            if isinstance(widget, QPushButton):
                if widget.isCheckable() and widget.isChecked() != source.isChecked():
                    widget.setChecked(source.isChecked())
                if source in (m.laserOutBut, m.modeBut) and widget.text() != source.text():
                    widget.setText(source.text())

        # Motors
        motors = [core.motors.brusher, core.motors.flame_io, core.motors.left_puller, core.motors.right_puller]
        motors_ready = all(motor.ok and motor.homed for motor in motors)
        if motors_ready:
            self.set_status(self.lbl_motors, "Ready", "ok")
        elif any(motor.error for motor in motors):
            self.set_status(self.lbl_motors, "Error", "error")
        elif m.initLoop_active or any(motor.ok for motor in motors):
            self.set_status(self.lbl_motors, "Initializing...", "busy")
        else:
            self.set_status(self.lbl_motors, "Not initialized", "off")

        # Laser
        if not m.laser.ok:
            self.set_status(self.lbl_laser, "Not connected", "off")
        elif m.laser.output:
            self.set_status(self.lbl_laser, "On", "ok")
        else:
            self.set_status(self.lbl_laser, "Connected, output off", "busy")

        # Power
        self.lbl_power.setText(f"{m.transpowIndSpin.value():.2f} dBm  (reference: {m.refpowIndSpin.value():.2f} dBm)")
        self.lbl_loss.setText(f"{m.lossIndSpin.value():.2f} dB")

        # Plan
        text = f"{m.total_to_pull:.2f} mm will be pulled, for a {m.setWaistLengthSpin.value():.2f} mm long waist."
        if m.profile_mode != "Standard":
            text = f"Using the {m.profile_mode.lower()} profile set in the Advanced view. " + text
        self.lbl_plan.setText(text)

        # Process
        self.lbl_pulled.setText(f"{m.totalPulledIndSpin.value():.3f} mm")
        self.lbl_diameter.setText(f"{m.waistDiamIndSpin.value():.2f} µm")
        self.progress.setValue(m.timeleftBar.value())

        if m.pullLoop_timer.isActive():
            self.lbl_time.setText(m.timeleftLabel.text())
            if core.stopping:
                self.set_status(self.lbl_stage, "Stopping...", "busy")
            elif core.pulling:
                self.set_status(self.lbl_stage, "Pulling", "error")
            elif core.flame_holding:
                self.set_status(self.lbl_stage, "Heating the fiber...", "busy")
            elif core.flame_approaching:
                self.set_status(self.lbl_stage, "Flame approaching...", "busy")
            else:
                self.set_status(self.lbl_stage, "Going to the start position...", "busy")
        else:
            self.lbl_time.setText("")
            if m.going_2start:
                self.set_status(self.lbl_stage, "Going to the start position...", "busy")
            elif not motors_ready:
                self.set_status(self.lbl_stage, "Motors not ready", "off")
            elif m.data.get_recording_duration() > 0:
                self.set_status(self.lbl_stage, "Finished", "ok")
            else:
                self.set_status(self.lbl_stage, "Ready", "ok")

        # Data
        if m.run_folder:
            self.lbl_data.setText(f"Last saved in: {os.path.basename(m.run_folder)}")
            self.lbl_data.setToolTip(m.run_folder)
        else:
            self.lbl_data.setText("Everything is saved automatically.")
            self.lbl_data.setToolTip(f"Data folder: {m.get_data_dir()}")
