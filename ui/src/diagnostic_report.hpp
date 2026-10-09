#pragma once

#include <QString>
#include <QStringList>

// Formats one finished lc_e_diag run. The dashboard cards are not updated from this text.
inline QString formatDiagnosticReport(const QString &stdoutText, const QString &stderrText, bool success,
                                      bool scalingUnknown = false) {
  QString result = QStringLiteral("РЕАЛЬНЫЙ SDO-СНИМОК (не циклическая OP-телеметрия)\n");
  result += QStringLiteral("Карточки сверху остаются демонстрацией и этим снимком не обновляются.\n");
  if (!success) {
    result += QStringLiteral("Диагностика не завершилась успешно; данные не считаются подтверждёнными.\n");
    result += (stderrText + QLatin1Char('\n') + stdoutText).right(1000);
    return result;
  }
  int kept = 0;
  const QStringList lines = stdoutText.split(QLatin1Char('\n'));
  for (const QString &line : lines) {
    const QString trimmed = line.trimmed();
    const bool keep = trimmed.startsWith(QLatin1String("state:")) ||
                      trimmed.startsWith(QLatin1String("name:")) ||
                      trimmed.startsWith(QLatin1String("vendor_id:")) ||
                      trimmed.startsWith(QLatin1String("product_code:")) ||
                      trimmed.startsWith(QLatin1String("revision:")) ||
                      trimmed.startsWith(QLatin1String("Error Code 0x603F:")) ||
                      trimmed.startsWith(QLatin1String("Status Word 0x6041:")) ||
                      trimmed.startsWith(QLatin1String("Actual position 0x6064:")) ||
                      trimmed.startsWith(QLatin1String("Actual velocity 0x606c:"), Qt::CaseInsensitive) ||
                      trimmed.startsWith(QLatin1String("Actual torque 0x6077:")) ||
                      trimmed.startsWith(QLatin1String("Mode display 0x6061:"));
    if (!keep) continue;
    result += trimmed + QLatin1Char('\n');
    ++kept;
  }
  if (kept == 0) {
    result += QStringLiteral("Программа завершилась без ожидаемых строк SDO. Показания не подтверждены.\n");
    result += stdoutText.right(1000);
    return result;
  }
  if (scalingUnknown) {
    result += QStringLiteral(
        "Связь есть, но 0x608F в этом снимке не прочитан. Командные единицы не подтверждены, движение закрыто.\n");
  }
  result += QStringLiteral("Мастер завершён. Двигатель не включался. WKC и циклический OP этим снимком не измеряются.");
  return result;
}
