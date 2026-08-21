# -*- coding: utf-8 -*-
# =============================================================================
# Created By   : Paulo Jarschel / Assistant
# Gleb Wataghin Physics Institute (IFGW), University of Campinas
# =============================================================================
"""
Interactive Loop Wizard Dialog for Fiber Taper Loop Fabrication.
Provides real-time position telemetry, micro-step jogging, press-and-hold
continuous slow creeping, and step-by-step guidance.
"""

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QDoubleSpinBox, QRadioButton, QButtonGroup,
    QMessageBox, QFrame, QTextEdit
)
from PyQt6.QtGui import QFont


class LoopWizardDialog(QDialog):
    """
    Interactive Dialog for guiding the fabrication and tuning of fiber taper loops.
    """

    def __init__(self, parent=None, core=None):
        super().__init__(parent)
        self.core = core
        self.setWindowTitle("Fiber Taper Loop Wizard")
        self.setMinimumSize(580, 680)
        self.resize(620, 720)

        # Store initial positions for delta tracking and reset functionality
        self.left_motor = self.core.motors.left_puller if self.core else None
        self.right_motor = self.core.motors.right_puller if self.core else None

        self.init_left_pos = self.left_motor.get_position() if self.left_motor else 0.0
        self.init_right_pos = self.right_motor.get_position() if self.right_motor else 0.0

        # Save default velocities to restore after creeping
        self.default_left_vel = self.left_motor.vel if self.left_motor else 40.0
        self.default_right_vel = self.right_motor.vel if self.right_motor else 40.0
        self.is_holding_continuous = False

        # Current step (0: Loosen, 1: Twist, 2: Tension)
        self.current_step = 0

        self.init_ui()

        # Telemetry update timer (50 ms / 20 Hz)
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        self.telemetry_timer.start(50)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # --- 1. Step-by-Step Guide Panel ---
        guide_group = QGroupBox("Fabrication Guide", self)
        guide_layout = QVBoxLayout(guide_group)

        # Step indicator header
        step_header_layout = QHBoxLayout()
        self.step_btn1 = QPushButton("1. Loosen (FW)", self)
        self.step_btn2 = QPushButton("2. Twist / Loop", self)
        self.step_btn3 = QPushButton("3. Tension (BW)", self)

        self.step_btn_group = QButtonGroup(self)
        self.step_btn_group.addButton(self.step_btn1, 0)
        self.step_btn_group.addButton(self.step_btn2, 1)
        self.step_btn_group.addButton(self.step_btn3, 2)

        for btn in (self.step_btn1, self.step_btn2, self.step_btn3):
            btn.setCheckable(True)
            step_header_layout.addWidget(btn)

        self.step_btn_group.idClicked.connect(self.set_step)
        self.step_btn1.setChecked(True)
        guide_layout.addLayout(step_header_layout)

        # Step instruction text
        self.guide_text = QTextEdit(self)
        self.guide_text.setReadOnly(True)
        self.guide_text.setMaximumHeight(85)
        self.guide_text.setStyleSheet("background-color: #f8f9fa; color: #212529; border: 1px solid #ced4da; border-radius: 4px; padding: 6px;")
        guide_layout.addWidget(self.guide_text)

        main_layout.addWidget(guide_group)

        # --- 2. Live Position & Telemetry Panel ---
        pos_group = QGroupBox("Live Motor Telemetry", self)
        pos_layout = QGridLayout(pos_group)
        pos_layout.setSpacing(8)

        bold_font = QFont()
        bold_font.setBold(True)

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
        self.lbl_left_pos.setStyleSheet("font-family: monospace; font-size: 13px;")
        self.lbl_left_delta = QLabel("+0.000 mm")
        self.lbl_left_delta.setStyleSheet("font-family: monospace; font-size: 13px;")
        self.lbl_left_status = QLabel("STOPPED")
        pos_layout.addWidget(self.lbl_left_pos, 1, 1)
        pos_layout.addWidget(self.lbl_left_delta, 1, 2)
        pos_layout.addWidget(self.lbl_left_status, 1, 3)

        # Right Puller row
        pos_layout.addWidget(QLabel("Right Puller:"), 2, 0)
        self.lbl_right_pos = QLabel("0.000 mm")
        self.lbl_right_pos.setStyleSheet("font-family: monospace; font-size: 13px;")
        self.lbl_right_delta = QLabel("+0.000 mm")
        self.lbl_right_delta.setStyleSheet("font-family: monospace; font-size: 13px;")
        self.lbl_right_status = QLabel("STOPPED")
        pos_layout.addWidget(self.lbl_right_pos, 2, 1)
        pos_layout.addWidget(self.lbl_right_delta, 2, 2)
        pos_layout.addWidget(self.lbl_right_status, 2, 3)

        # Net Slack / Separation Change
        pos_layout.addWidget(QLabel("Total Slack Added:"), 3, 0)
        self.lbl_total_slack = QLabel("0.000 mm")
        self.lbl_total_slack.setStyleSheet("font-family: monospace; font-weight: bold; font-size: 13px; color: #0d6efd;")
        pos_layout.addWidget(self.lbl_total_slack, 3, 1, 1, 3)

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
        step_config_layout.addWidget(self.spin_step_size)

        # Quick preset buttons
        step_config_layout.addWidget(QLabel("Presets:"))
        for preset_val, preset_label in [(0.01, "10µm"), (0.05, "50µm"), (0.1, "0.1mm"), (0.5, "0.5mm"), (1.0, "1mm")]:
            p_btn = QPushButton(preset_label, self)
            p_btn.setMaximumWidth(52)
            p_btn.clicked.connect(lambda _, v=preset_val: self.spin_step_size.setValue(v))
            step_config_layout.addWidget(p_btn)
        step_config_layout.addStretch()
        step_layout.addLayout(step_config_layout)

        # Step Jog Buttons
        jog_btn_layout = QHBoxLayout()
        self.btn_step_loosen = QPushButton("◀◀ Step FW (+ Loosen)", self)
        self.btn_step_loosen.setStyleSheet("padding: 8px; font-weight: bold; background-color: #e7f1ff; border: 1px solid #b6d4fe; border-radius: 4px;")
        self.btn_step_loosen.clicked.connect(self.do_step_loosen)

        self.btn_step_tighten = QPushButton("Step BW (- Tighten) ▶▶", self)
        self.btn_step_tighten.setStyleSheet("padding: 8px; font-weight: bold; background-color: #fff3cd; border: 1px solid #ffe69c; border-radius: 4px;")
        self.btn_step_tighten.clicked.connect(self.do_step_tighten)

        jog_btn_layout.addWidget(self.btn_step_loosen)
        jog_btn_layout.addWidget(self.btn_step_tighten)
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
        creep_config_layout.addWidget(self.spin_creep_vel)

        for c_val, c_lbl in [(0.01, "0.01mm/s"), (0.02, "0.02mm/s"), (0.05, "0.05mm/s"), (0.1, "0.1mm/s")]:
            c_btn = QPushButton(c_lbl, self)
            c_btn.setMaximumWidth(70)
            c_btn.clicked.connect(lambda _, v=c_val: self.spin_creep_vel.setValue(v))
            creep_config_layout.addWidget(c_btn)
        creep_config_layout.addStretch()
        creep_layout.addLayout(creep_config_layout)

        # Press & Hold buttons
        hold_btn_layout = QHBoxLayout()
        self.btn_hold_loosen = QPushButton("▼ HOLD to Loosen (FW +)", self)
        self.btn_hold_loosen.setStyleSheet("padding: 10px; font-weight: bold; background-color: #cfe2ff; border: 1px solid #9ec5fe; border-radius: 4px;")
        self.btn_hold_loosen.pressed.connect(self.on_hold_loosen_pressed)
        self.btn_hold_loosen.released.connect(self.on_hold_released)

        self.btn_hold_tighten = QPushButton("▲ HOLD to Tighten (BW -)", self)
        self.btn_hold_tighten.setStyleSheet("padding: 10px; font-weight: bold; background-color: #ffe69c; border: 1px solid #ffda6a; border-radius: 4px;")
        self.btn_hold_tighten.pressed.connect(self.on_hold_tighten_pressed)
        self.btn_hold_tighten.released.connect(self.on_hold_released)

        hold_btn_layout.addWidget(self.btn_hold_loosen)
        hold_btn_layout.addWidget(self.btn_hold_tighten)
        creep_layout.addLayout(hold_btn_layout)

        main_layout.addWidget(creep_group)

        # --- 5. Safety & Actions Bottom Bar ---
        action_layout = QHBoxLayout()

        self.btn_reset_pos = QPushButton("Reset to Start Pos", self)
        self.btn_reset_pos.setToolTip("Return both motors to the position when the wizard opened")
        self.btn_reset_pos.clicked.connect(self.do_reset_positions)

        self.btn_stop = QPushButton("STOP", self)
        self.btn_stop.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; padding: 6px 14px; border-radius: 4px;")
        self.btn_stop.clicked.connect(self.do_emergency_stop)

        self.btn_done = QPushButton("Done / Close", self)
        self.btn_done.setStyleSheet("font-weight: bold; padding: 6px 14px;")
        self.btn_done.clicked.connect(self.accept)

        action_layout.addWidget(self.btn_reset_pos)
        action_layout.addWidget(self.btn_stop)
        action_layout.addStretch()
        action_layout.addWidget(self.btn_done)

        main_layout.addLayout(action_layout)

        # Update initial step instructions
        self.update_step_guide()

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
        if not self.left_motor or not self.right_motor:
            return

        l_pos = self.left_motor.get_position()
        r_pos = self.right_motor.get_position()

        dl = l_pos - self.init_left_pos
        dr = r_pos - self.init_right_pos
        total_slack = dl + dr

        self.lbl_left_pos.setText(f"{l_pos:.3f} mm")
        self.lbl_right_pos.setText(f"{r_pos:.3f} mm")

        self.lbl_left_delta.setText(f"{'+' if dl >= 0 else ''}{dl:.3f} mm")
        self.lbl_right_delta.setText(f"{'+' if dr >= 0 else ''}{dr:.3f} mm")

        self.lbl_total_slack.setText(f"{'+' if total_slack >= 0 else ''}{total_slack:.3f} mm")

        # Update motion indicators
        if self.left_motor.motor_moving():
            self.lbl_left_status.setText("MOVING")
            self.lbl_left_status.setStyleSheet("color: #0d6efd; font-weight: bold;")
        else:
            self.lbl_left_status.setText("STOPPED")
            self.lbl_left_status.setStyleSheet("color: #198754;")

        if self.right_motor.motor_moving():
            self.lbl_right_status.setText("MOVING")
            self.lbl_right_status.setStyleSheet("color: #0d6efd; font-weight: bold;")
        else:
            self.lbl_right_status.setText("STOPPED")
            self.lbl_right_status.setStyleSheet("color: #198754;")

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

        # Safety boundary checks
        if not (self.left_motor.safe_range[0] <= target_l <= self.left_motor.safe_range[1]):
            QMessageBox.warning(self, "Safety Limit", f"Left puller target {target_l:.3f} mm is outside safe range [{self.left_motor.safe_range[0]}, {self.left_motor.safe_range[1]}].")
            return
        if not (self.right_motor.safe_range[0] <= target_r <= self.right_motor.safe_range[1]):
            QMessageBox.warning(self, "Safety Limit", f"Right puller target {target_r:.3f} mm is outside safe range [{self.right_motor.safe_range[0]}, {self.right_motor.safe_range[1]}].")
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

    def closeEvent(self, event):
        """Ensure all motors are stopped, velocities restored, and timer killed upon closing."""
        self.do_emergency_stop()
        if self.left_motor:
            self.left_motor.set_velocity(self.default_left_vel)
        if self.right_motor:
            self.right_motor.set_velocity(self.default_right_vel)
        self.telemetry_timer.stop()
        event.accept()
