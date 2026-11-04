# passive-flight-core

Расчёт невозмущённого и возмущённого продольного движения авиационной бомбы
ФАБ-1500Т. Аэродинамические коэффициенты берутся из табличных данных
Постникова и Чуйко (1979); расчёт аэродинамики по произвольной геометрии в
проект не входит.

Ядро не зависит от GUI. Для внешних программ предусмотрен C ABI, для
SimInTech — четыре тонких DLL-адаптера.

## Поддерживаемый объект

- стабильный ID: `FAB_1500T_POSTNIKOV_1979`;
- индекс в C API и блоках SimInTech: `0`;
- масса: 1519 кг;
- площадь: площадь миделя;
- характерная длина момента: длина бомбы;
- коэффициенты: `Cx0(M)`, `Cx^(alpha^2)(M)`, `Cy^alpha(M)`,
  `mz^alpha(M)` и `mz^(omega_bar)(M)`;
- нормировка угловой скорости: `omega_bar_z = omega_z * Lref / V` без
  дополнительного множителя `1/2`.

## Сборка и тесты

Нужны CMake 3.20 или новее и компиляторы C11/C++20. На Windows сборка MinGW
создаёт как основную C API DLL, так и адаптеры SimInTech.

```sh
cmake -S . -B out/build/debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build out/build/debug --parallel 4
ctest --test-dir out/build/debug --output-on-failure
```

## Библиотеки

Основные цели CMake:

- `passive_flight_core` — статическое C++-ядро;
- `passive_flight_c_api` — C-совместимая DLL с номинальным и возмущённым
  расчётом;
- `passive_flight_simintech` — итог невозмущённого расчёта;
- `passive_flight_trajectory_simintech` — невозмущённая траектория;
- `passive_flight_perturbed_simintech` — итог возмущённого расчёта;
- `passive_flight_perturbed_trajectory_simintech` — возмущённая траектория.

При установке адаптера SimInTech рядом с ним должна находиться
`passive_flight_c_api.dll` (для MinGW имя файла может иметь префикс `lib`).
Публичные C-заголовки расположены в `include/passive_flight_c_api`, код блоков
SimInTech — в `src_simintech`.

## Архитектура

`ObjectPassport` хранит параметры и происхождение данных ФАБ-1500Т,
`ObjectRegistry` выполняет проверку и выбор объекта, а
`TabulatedAerodynamicModel` интерполирует пять таблиц по числу Маха.
`NominalLongitudinalDynamics` вычисляет правую часть невозмущённого движения.
`PerturbedLongitudinalDynamics` получает матрицу линейной модели численным
дифференцированием той же правой части, поэтому обе ветви используют один набор
физики и коэффициентов. C API и блоки SimInTech не содержат собственной
аэродинамической модели.

Правила единиц, знаков и работы с источниками зафиксированы в `AGENTS.md`.
Прохождение программных тестов не заменяет физическую валидацию по независимым
баллистическим данным.
