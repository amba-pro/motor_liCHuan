#pragma once

#include <QString>

inline QString industrialStyle() {
  return QStringLiteral(
      "QMainWindow, QWidget { background: #12171c; color: #d7e0e8; "
      "font-family: Ubuntu, 'Noto Sans', 'DejaVu Sans'; font-size: 13px; }"
      "QTabWidget::pane { border: 1px solid #2c3640; }"
      "QTabBar::tab { background: #1c242c; color: #b7c3ce; padding: 10px 18px; }"
      "QTabBar::tab:selected { background: #243038; color: #f4f7f8; border-bottom: 2px solid #3d9a78; }"
      "QFrame#telemetryCard { background: #1a2229; border: 1px solid #31404a; border-radius: 6px; }"
      "QGroupBox { border: 1px solid #31404a; margin-top: 14px; border-radius: 6px; background: #1a2229; "
      "padding: 12px; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #9aabba; }"
      "QPushButton { background: #243038; border: 1px solid #3d4e5a; padding: 8px 14px; border-radius: 4px; }"
      "QPushButton:hover { background: #2d3c46; }"
      "QPushButton:disabled { color: #6d7a84; background: #1a2127; }"
      "QPushButton#demoStartButton { background: #1e4d3d; }"
      "QPushButton#demoStopButton { background: #5a341f; }"
      "QDoubleSpinBox { background: #0f1418; border: 1px solid #3d4e5a; padding: 4px; }"
      "QLabel#positionValue { font-size: 34px; font-weight: 600; color: #f4f7f8; }"
      "QLabel#demoBanner { background: #3d3418; color: #f0d48a; padding: 8px 12px; }"
      "QLabel#estopWarning { color: #f0d48a; }"
      "QLabel#brakeState { color: #f0d48a; }"
      "QLabel#readOnlyDiagnosticResult { background: #10161b; color: #d5dde4; "
      "border: 1px solid #3d4e5a; padding: 8px; }");
}
