// Colores y hoja de estilos del dashboard (tema oscuro).
#pragma once

#include <QColor>
#include <QString>

namespace theme {

inline const QColor kBg(11, 15, 20);
inline const QColor kCanvas(9, 12, 16);
inline const QColor kGridLine(22, 30, 40);
inline const QColor kText(229, 231, 235);
inline const QColor kMuted(139, 152, 168);
inline const QColor kAccent(34, 211, 238);
inline const QColor kPath(168, 85, 247);
inline const QColor kMapFree(30, 41, 59);
inline const QColor kMapOcc(226, 232, 240);
inline const QColor kGreen(34, 197, 94);
inline const QColor kAmber(245, 158, 11);
inline const QColor kRed(239, 68, 68);

inline QString styleSheet()
{
  return R"(
* { color: #e5e7eb; font-family: "Ubuntu", "DejaVu Sans", sans-serif; font-size: 13px; }
QMainWindow, #root { background: #0b0f14; }

#header { background: #0f151c; border-bottom: 1px solid #1f2a37; }
#brand { font-size: 17px; font-weight: 700; color: #f8fafc; }
#brandSub { color: #8b98a8; font-size: 12px; }
#timer { font-family: "Ubuntu Mono", "DejaVu Sans Mono", monospace; font-size: 30px; font-weight: 700; color: #f8fafc; }
#timerMax { color: #8b98a8; font-family: "Ubuntu Mono", monospace; font-size: 15px; }

#modeChip { border-radius: 11px; padding: 3px 12px; font-weight: 700; font-size: 12px; }
#modeChip[mode="IDLE"]    { background: #1f2937; color: #cbd5e1; }
#modeChip[mode="MAPPING"] { background: #052e1a; color: #4ade80; border: 1px solid #166534; }
#modeChip[mode="PAUSED"]  { background: #2d1f05; color: #fbbf24; border: 1px solid #92400e; }

#card { background: #111821; border: 1px solid #1f2a37; border-radius: 10px; }
#cardTitle { font-weight: 700; font-size: 13px; color: #f1f5f9; }
#cardHint { color: #64748b; font-size: 11px; }
#sideTitle { color: #64748b; font-size: 11px; font-weight: 700; letter-spacing: 1px; }

#statBox { background: #0d131a; border: 1px solid #1a2430; border-radius: 8px; }
#statLabel { color: #8b98a8; font-size: 11px; }
#statValue { font-family: "Ubuntu Mono", "DejaVu Sans Mono", monospace; font-size: 16px; font-weight: 700; color: #f8fafc; }

#topicName { font-family: "Ubuntu Mono", monospace; font-size: 12px; color: #cbd5e1; }
#topicHz { font-family: "Ubuntu Mono", monospace; font-size: 12px; color: #8b98a8; }

QPushButton {
  background: #18212c; border: 1px solid #263445; border-radius: 8px;
  padding: 8px 12px; font-weight: 600; text-align: left;
}
QPushButton:hover { background: #1f2b39; border-color: #33475e; }
QPushButton:pressed { background: #141b24; }
QPushButton:disabled { color: #4b5563; background: #12181f; border-color: #1a222c; }
QPushButton:checked { background: #082f49; border-color: #0e7490; color: #67e8f9; }

#primary { background: #16a34a; border: none; color: white; font-size: 15px; padding: 12px; text-align: center; }
#primary:hover { background: #22c55e; }
#primary:disabled { background: #1a222c; color: #4b5563; }
#primary[state="pause"] { background: #d97706; }
#primary[state="pause"]:hover { background: #f59e0b; }
#danger { background: transparent; border: 1px solid #7f1d1d; color: #fca5a5; text-align: center; }
#danger:hover { background: #2a0f0f; }

#tool { padding: 3px 9px; font-size: 12px; font-weight: 500; border-radius: 6px; text-align: center; }

QProgressBar { background: #1a2430; border: none; border-radius: 3px; max-height: 6px; }
QProgressBar::chunk { background: #22d3ee; border-radius: 3px; }
QProgressBar[warn="1"]::chunk { background: #f59e0b; }
QProgressBar[warn="2"]::chunk { background: #ef4444; }

#toast { border-radius: 8px; padding: 8px 12px; font-weight: 600; }
#toast[kind="ok"]   { background: #052e1a; color: #86efac; border: 1px solid #166534; }
#toast[kind="warn"] { background: #2d1f05; color: #fcd34d; border: 1px solid #92400e; }
#toast[kind="err"]  { background: #2a0f0f; color: #fca5a5; border: 1px solid #7f1d1d; }
#toast[kind="info"] { background: #0c1a26; color: #7dd3fc; border: 1px solid #0e7490; }

QComboBox { background: #18212c; border: 1px solid #263445; border-radius: 6px; padding: 4px 8px; }
QComboBox QAbstractItemView { background: #111821; border: 1px solid #263445; selection-background-color: #082f49; }
QPlainTextEdit { background: #0d131a; border: 1px solid #1a2430; border-radius: 6px; font-family: "Ubuntu Mono", monospace; font-size: 11px; color: #cbd5e1; }
QSplitter::handle { background: #0b0f14; }
QScrollArea { border: none; background: #0b0f14; }
#side { background: #0b0f14; }
QToolTip { background: #111821; color: #e5e7eb; border: 1px solid #263445; }
QMessageBox { background: #111821; }
)";
}

}  // namespace theme
