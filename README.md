# passive-flight-core

C++-ядро для моделирования невозмущённого и возмущённого продольного движения
пассивного летательного аппарата. Ядро работает без GUI; внешние приложения
подключаются через C API, а SimInTech — через отдельные адаптеры.

Правила работы с кодом, единицами и научными источниками находятся в
[AGENTS.md](AGENTS.md). Порядок начала инженерной сессии описан в
[CODEX_START_PROMPT.md](docs/agent/CODEX_START_PROMPT.md).

## Сборка и тесты

Требуются CMake не ниже 3.20, компиляторы C11 и C++20. Для команд ниже нужен
Ninja; `cmake`, `ctest`, `ninja` и компиляторы должны быть доступны в `PATH`.
В Windows для сборки адаптеров SimInTech требуется MinGW.

```sh
cmake -S . -B out/build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build out/build/debug --parallel 4
ctest --test-dir out/build/debug --output-on-failure
```

Запускайте команды из корня репозитория. Используйте отдельный каталог для
каждого компилятора и конфигурации. Локальные результаты сборки можно исключить
из Git строкой `/out/` в `.git/info/exclude`.

Настройка PowerShell с toolchain CLion, группы тестов и проверенная исходная
конфигурация приведены в [руководстве разработчика](docs/development.md).

## Архитектура

| Слой | Основные интерфейсы | Ответственность |
|---|---|---|
| Данные объекта | [ObjectModel](include/passive_flight/ObjectModel.hpp), [ObjectPassport](include/passive_flight/ObjectPassport.hpp) | Числовые параметры и отдельные записи об их происхождении |
| Реестр | [ObjectRegistry](include/passive_flight/ObjectRegistry.hpp) | Валидация и регистрация паспортов, выбор объекта по стабильному ID |
| Аэродинамика | [AerodynamicModel](include/passive_flight/Aerodynamics.hpp), [TabulatedAerodynamicModel](include/passive_flight/TabulatedAerodynamicModel.hpp) | Общий интерфейс предварительной геометрической и табличной моделей |
| Адаптация объекта | [ObjectAerodynamicsAdapter](include/passive_flight/ObjectAerodynamicsAdapter.hpp) | Создание аэродинамической модели из данных объекта |
| Атмосфера | [StandardAtmosphere](include/passive_flight/Atmosphere.hpp) | Параметры воздуха по высоте |
| Динамика | [NominalLongitudinalDynamics](include/passive_flight/NominalLongitudinalDynamics.hpp), [PerturbedLongitudinalDynamics](include/passive_flight/PerturbedLongitudinalDynamics.hpp) | Невозмущённое движение и численная линеаризация возмущений |
| Интегрирование и результаты | [ForwardEulerSimulator](include/passive_flight/ForwardEulerSimulator.hpp), [PerturbedTrajectorySimulator](include/passive_flight/PerturbedTrajectorySimulator.hpp), [TrajectoryAnalysis](include/passive_flight/TrajectoryAnalysis.hpp) | Построение и анализ траектории; отдельные модули анализа возмущений и CSV |
| C ABI | [include/passive_flight_c_api](include/passive_flight_c_api) и [src_c_api](src_c_api) | C-совместимые структуры, коды результата и буферы внешнего приложения |
| SimInTech | [src_simintech](src_simintech) | Адаптеры поверх C API; собираются при `WIN32 AND MINGW` |

Состав целей и условия их доступности определены в
[CMakeLists.txt](CMakeLists.txt). Статическая библиотека называется
`passive_flight_core`, разделяемая библиотека C API — `passive_flight_c_api`.

## Что уже доступно для собственной компоновки

- C++-интерфейсы принимают `ObjectModel`; паспорт можно добавить в собственный
  экземпляр `ObjectRegistry` через `add()` после валидации.
- `ObjectModel` содержит массово-инерционные параметры, опорную геометрию,
  корпус, крыло и стабилизатор. Наличие поля не означает, что оно уже участвует
  в предварительном аэродинамическом расчёте.
- Табличная и предварительная геометрическая модели реализуют общий
  `AerodynamicModel`, поэтому способ получения коэффициентов отделён от динамики.

**Геометрическая ветка пока предварительная.** Существующий адаптер подключает
вспомогательные таблицы базового примера. Полного, проверенного по источникам
расчёта всех необходимых характеристик только по пользовательской геометрии
пока нет.

**C API пока не принимает новую геометрию или новый паспорт.** Его функции
выбирают объект из внутренних стандартных реестров. Программная возможность
создать отдельный реестр в C++ не является функцией регистрации объекта в DLL.

## Источники и состояние проверки

Локальные материалы расположены в [docs/references](docs/references):

- `Lebedev_Chernobrovkin_1973.djvu`;
- `Postnikov_Chuyko_1979.pdf`;
- `Voenmeh_aerodynamics.pdf`;
- `layout_reference.png` — концептуальная схема исходных размеров.

Этот перечень фиксирует наличие файлов. Он не подтверждает прочтение всех
страниц, корректность оцифровки или полноту исходных данных. Перед реализацией
физической зависимости требуется изучить соответствующий фрагмент источника
и записать его происхождение, единицы, соглашения и область применимости
по правилам `AGENTS.md`.

В [руководстве разработчика](docs/development.md) записан результат проверки
программной сборки и тестов. Прохождение тестов само по себе не подтверждает
физическую валидацию модели по независимым данным.
