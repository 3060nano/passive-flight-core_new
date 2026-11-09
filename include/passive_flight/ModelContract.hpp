#pragma once

#include "passive_flight/Types.hpp"

#include <string>
#include <vector>

namespace passive_flight {

    /// Описание одного нарушения контракта входных данных.
    struct ValidationIssue {
        /// Стабильный путь к некорректному полю модели или запроса.
        std::string field;

        /// Диагностическое сообщение на английском языке для журналирования.
        std::string message;
    };

    /// Последовательность обнаруженных нарушений контракта.
    using ValidationIssues = std::vector<ValidationIssue>;

    /*
     * Формирует начальное состояние горизонтального сброса:
     *
     *     t0      = 0;
     *     x0      = 0;
     *     H0      = заданная высота;
     *     V0      = заданная скорость;
     *     Theta0  = 0;
     *     theta0  = 0;
     *     omegaZ0 = 0;
     *     alpha0  = 0.
     *
     * При недопустимых исходных данных функция выбрасывает
     * std::invalid_argument.
     *
     * @param release Высота в метрах и скорость в м/с для горизонтального сброса.
     * @return Начальное состояние в единицах СИ с нулевыми углами и координатой x.
     * @throws std::invalid_argument Если высота или скорость некорректны.
     */
    [[nodiscard]] State makeHorizontalReleaseState(
        const ReleaseConditions& release
    );

    /*
     * Формирует краткий результат из полного результата моделирования.
     * @param result Полный результат с конечным состоянием в единицах СИ.
     * @return Параметры конечного состояния и причина остановки.
     */
    [[nodiscard]] SimulationSummary summarize(
        const SimulationResult& result
    );

    /*
     * Проверяет конечность и положительность высоты и скорости сброса.
     * @param release Проверяемые значения высоты, м, и скорости, м/с.
     * @return Список всех найденных нарушений; пустой список означает успех.
     */
    [[nodiscard]] ValidationIssues validate(
        const ReleaseConditions& release
    );

    /*
     * Проверяет полный запрос на моделирование, включая идентификатор объекта.
     * @param request Запрос с идентификатором и условиями сброса.
     * @return Список всех найденных нарушений.
     */
    [[nodiscard]] ValidationIssues validate(
        const SimulationRequest& request
    );

    /*
     * Проверяет отсутствие нарушений контракта.
     * @param issues Список результатов валидации.
     * @return `true`, если список пуст; иначе `false`.
     */
    [[nodiscard]] bool isValid(
        const ValidationIssues& issues
    ) noexcept;

    /*
     * Возвращает текстовое название причины остановки.
     *
     * Текстовое представление используется при журналировании,
     * автоматическом тестировании и преобразовании в C API.
     * @param reason Перечислимое значение причины остановки.
     * @return Указатель на статическую ASCII-строку; освобождение не требуется.
     */
    [[nodiscard]] const char* terminationReasonName(
        TerminationReason reason
    ) noexcept;

} // namespace passive_flight
