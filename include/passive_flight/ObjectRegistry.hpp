#pragma once

#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/Types.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace passive_flight {

/**
 * Краткое описание расчётного объекта для внешних программных интерфейсов.
 *
 * Структура передаёт идентификационные сведения, необходимые библиотечному
 * интерфейсу, блокам SimInTech и пользовательским средствам выбора объекта;
 * параметры математической модели хранятся отдельно в `ObjectPassport`.
 */
struct ObjectDescriptor {
    /// Стабильный уникальный идентификатор объекта.
    std::string id;

    /// Человекочитаемое название объекта.
    std::string displayName;

    /// Версия числовой модели.
    std::string modelVersion;
};

/**
 * Реестр расчётных объектов.
 *
 * Каждый объект имеет уникальный строковый идентификатор.
 * Один и тот же решатель может использоваться для любого
 * объекта, зарегистрированного в реестре.
 */
class ObjectRegistry {
public:
    /// Создаёт пустой реестр расчётных объектов.
    ObjectRegistry() = default;

    /**
     * Добавляет паспорт объекта.
     *
     * Перед добавлением выполняется полная валидация.
     *
     * @throws std::invalid_argument:
     * - если паспорт некорректен;
     * - если идентификатор уже зарегистрирован.
     * @param passport Паспорт, перемещаемый во внутреннее хранилище реестра.
     */
    void add(
        ObjectPassport passport
    );

    /**
     * Проверяет наличие объекта.
     * @param objectId Стабильный идентификатор искомой модели.
     * @return `true`, если объект зарегистрирован; иначе `false`.
     */
    [[nodiscard]]
    bool contains(
        const std::string& objectId
    ) const noexcept;

    /**
     * Возвращает паспорт объекта.
     *
     * Если объект не найден, возвращается nullptr.
     * @param objectId Стабильный идентификатор искомой модели.
     * @return Указатель на паспорт либо `nullptr`; указатель действует до изменения реестра.
     */
    [[nodiscard]]
    const ObjectPassport* findPassport(
        const std::string& objectId
    ) const noexcept;

    /**
     * Возвращает только расчётную модель объекта.
     *
     * Если объект не найден, возвращается nullptr.
     * @param objectId Стабильный идентификатор искомой модели.
     * @return Указатель на ObjectModel либо `nullptr`; указатель действует до изменения реестра.
     */
    [[nodiscard]]
    const ObjectModel* findObject(
        const std::string& objectId
    ) const noexcept;

    /**
     * Возвращает список доступных объектов
     * в порядке их регистрации.
     * @return Копия кратких описаний объектов.
     */
    [[nodiscard]]
    std::vector<ObjectDescriptor>
    descriptors() const;

    /**
     * Количество объектов в реестре.
     * @return Число зарегистрированных паспортов, безразмерный счётчик.
     */
    [[nodiscard]]
    std::size_t size() const noexcept;

    /**
     * Проверяет, что реестр пуст.
     * @return `true`, если в реестре нет объектов; иначе `false`.
     */
    [[nodiscard]]
    bool empty() const noexcept;

    /**
     * Выполняет моделирование объекта, выбранного
     * через SimulationRequest.objectId.
     *
     * Если идентификатор неизвестен, возвращает результат
     * с причиной TerminationReason::InvalidInput.
     * @param request Идентификатор объекта и условия сброса в СИ.
     * @param options Настройки интегрирования и сохранения истории.
     * @return Полный результат расчёта либо результат с признаком ошибки входа.
     */
    [[nodiscard]]
    SimulationResult simulate(
        const SimulationRequest& request,
        const SimulationOptions& options = {}
    ) const;

private:
    /// Паспорта объектов в порядке их регистрации.
    std::vector<ObjectPassport> passports_;

    /// Соответствие стабильного идентификатора индексу в passports_.
    std::unordered_map<
        std::string,
        std::size_t
    > indexById_;
};

/**
 * Создаёт стандартный реестр ядра.
 *
 * В текущей конфигурации реестр содержит три варианта ФАБ-1500Т:
 *
 * - контрольную модель по Постникову—Чуйко;
 * - расчётную модификацию интерполированных производных;
 * - расчётную планирующую модификацию.
 *
 * @return Реестр штатно доступных паспортов объектов.
 */
[[nodiscard]]
ObjectRegistry makeDefaultObjectRegistry();

} // namespace passive_flight
