#pragma once

#include "passive_flight/ModelContract.hpp"
#include "passive_flight/ObjectModel.hpp"

#include <string>
#include <variant>
#include <vector>

namespace passive_flight {
    /// Категория происхождения параметра числовой модели.
    enum class ParameterStatus {
        /// Значение задано требованиями к модели или интерфейсу.
        Requirement,

        /// Значение непосредственно перенесено из указанного источника.
        SourceDocument,

        /// Значение вычислено из других документированных параметров.
        Derived,

        /// Предварительное значение, требующее последующего подтверждения.
        Provisional,

        /// Значение получено из вспомогательной таблицы.
        AuxiliaryTable
    };

    /// Числовое либо текстовое значение паспортного параметра.
    using ParameterValue = std::variant<double, std::string>;

    /// Запись о значении, единице измерения и происхождении одного параметра.
    struct ParameterRecord {
        /// Стабильный иерархический путь параметра внутри ObjectModel.
        std::string path;

        /// Числовое значение в указанной единице либо текстовое значение.
        ParameterValue value;

        /// Обозначение единицы измерения; `1` — безразмерная величина, `-` — текст.
        std::string unit;

        /// Категория происхождения значения.
        ParameterStatus status{ParameterStatus::Provisional};

        /// Идентификатор документа, таблицы или архитектурного решения-источника.
        std::string source;

        /// Пояснение физического смысла или способа получения значения.
        std::string note;
    };

    /*
     * Полный паспорт объекта.
     *
     * object содержит числовую модель для решателя.
     * parameters содержит происхождение основных
     * паспортных величин и аэродинамического набора.
     */
    struct ObjectPassport {
        /// Числовая модель, непосредственно используемая решателями.
        ObjectModel object;

        /// Записи о происхождении основных параметров модели.
        std::vector<ParameterRecord> parameters;
    };

    /*
     * Создаёт полный контрольный паспорт ФАБ-1500Т
     * с готовыми табличными аэродинамическими
     * характеристиками.
     * @return Валидируемый паспорт с числовой моделью и записями происхождения.
     */
    [[nodiscard]]
    ObjectPassport makeFab1500TPostnikovPassport();

    /*
     * Создаёт паспорт для модифицированной модели ФАБ-1500Т
     * (новые Cy/mz из предоставленной таблицы + Cx0 * 1.05).
     */
    [[nodiscard]]
    ObjectPassport makeFab1500TModifiedPassport();

    [[nodiscard]]
    ObjectPassport makeFab1500TGlidePassport();

    /*
     * Ищет параметр по стабильному пути.
     * @param passport Паспорт, в котором выполняется поиск.
     * @param path Полный путь параметра.
     * @return Указатель на запись либо `nullptr`; указатель действует до изменения passport.
     */
    [[nodiscard]]
    const ParameterRecord *findParameter(
        const ObjectPassport &passport,
        const std::string &path
    ) noexcept;

    /*
     * Проверяет числовую модель объекта.
     *
     * Набор обязательных полей зависит от
     * aerodynamicModelType.
     * @param object Проверяемая числовая модель.
     * @return Все обнаруженные нарушения контракта.
     */
    [[nodiscard]]
    ValidationIssues validate(
        const ObjectModel &object
    );

    /*
     * Проверяет паспорт вместе с происхождением
     * основных параметров.
     * @param passport Проверяемый паспорт объекта.
     * @return Нарушения числовой модели и записей происхождения.
     */
    [[nodiscard]]
    ValidationIssues validate(
        const ObjectPassport &passport
    );

    /// Возвращает стабильное текстовое имя категории происхождения.
    /// @param status Категория происхождения параметра.
    /// @return Указатель на статическую ASCII-строку; освобождение не требуется.
    [[nodiscard]]
    const char *parameterStatusName(
        ParameterStatus status
    ) noexcept;

    /// Возвращает стабильное текстовое имя типа аэродинамической модели.
    /// @param type Тип способа задания аэродинамики.
    /// @return Указатель на статическую ASCII-строку; освобождение не требуется.
    [[nodiscard]]
    const char *aerodynamicModelTypeName(
        AerodynamicModelType type
    ) noexcept;
} // namespace passive_flight
