#include "safety_gate.hpp"

std::vector<std::string> SafetyGate::blockers() const {
  return {
      "Аппаратная аварийная остановка не подтверждена. Программный СТОП её не заменяет.",
      "Тормоз питается отдельно от 24 В и отпущен. Отключение серво не включает тормоз.",
      "Фиксация, свободный вал и пределы хода не подтверждены для команды из этого окна.",
      "Цикл 1 мс не принят: на 30 с был пропуск срока 1,78 мс. Порог 250 мкс не ослаблен.",
      "Реальное включение и движение в этой сборке не передаются на привод.",
  };
}

bool SafetyGate::realEnableAllowed() const { return blockers().empty(); }

bool SafetyGate::realMoveAllowed() const { return blockers().empty(); }
