# Руководство разработчика

## Сначала проверьте рабочее дерево

Прочитайте [AGENTS.md](../AGENTS.md), затем проверьте ветку, индекс и локальные
правки. Подготовленные к коммиту изменения могут отличаться от рабочей копии.

```sh
git branch --show-current
git status --short
git diff
git diff --cached
```

Архитектура описана в [README](../README.md), а правила инженерной сессии — в
[стартовом промпте](agent/CODEX_START_PROMPT.md). Решения, которые изменяют
контракты или научные соглашения, следует фиксировать под `docs/agent/decisions/`.

## Toolchain

[CMakeLists.txt](../CMakeLists.txt) требует CMake 3.20+, C11 и C++20.
Все команды сборки ниже выполняются из корня репозитория.

Для обычного терминала заранее настройте `PATH` для CMake, CTest, Ninja и
компиляторов выбранного toolchain. Наличие работающего профиля CLion не означает,
что его инструменты доступны из отдельной PowerShell-консоли.

### Windows и инструменты из существующего профиля CLion

Если проект уже настроен в CLion с MinGW и Ninja, фактические пути можно взять
из `cmake-build-debug/CMakeCache.txt`. Скрипт ниже проверяет наличие файлов,
добавляет их каталоги в `PATH` только текущего процесса и создаёт отдельную сборку.
При другом имени профиля укажите его каталог в `$cacheFile`.

```powershell
$cacheFile = 'cmake-build-debug/CMakeCache.txt'
$cacheLines = Get-Content -LiteralPath $cacheFile

# Читает сохранённый путь инструмента из существующего CMake-профиля.
function Get-CachedToolPath([string] $key) {
    $pattern = '^' + [regex]::Escape($key) + ':[^=]+=(.+)$'
    foreach ($line in $cacheLines) {
        if ($line -match $pattern) {
            $toolPath = $Matches[1].Trim()
            if (-not (Test-Path -LiteralPath $toolPath -PathType Leaf)) {
                throw "Инструмент отсутствует: $toolPath"
            }
            return $toolPath
        }
    }
    throw "В кэше отсутствует $key"
}

$cmakeExe = Get-CachedToolPath 'CMAKE_COMMAND'
$ccExe = Get-CachedToolPath 'CMAKE_C_COMPILER'
$cxxExe = Get-CachedToolPath 'CMAKE_CXX_COMPILER'
$ninjaExe = Get-CachedToolPath 'CMAKE_MAKE_PROGRAM'
$ctestExe = Join-Path (Split-Path -Parent $cmakeExe) 'ctest.exe'
$toolDirectories = @(
    (Split-Path -Parent $cmakeExe)
    (Split-Path -Parent $ccExe)
    (Split-Path -Parent $cxxExe)
    (Split-Path -Parent $ninjaExe)
)
$env:PATH = ($toolDirectories -join ';') + ';' + $env:PATH
$buildDir = 'out/build/debug'

& $cmakeExe -S . -B $buildDir -G Ninja `
    -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON `
    "-DCMAKE_C_COMPILER=$ccExe" "-DCMAKE_CXX_COMPILER=$cxxExe" `
    "-DCMAKE_MAKE_PROGRAM=$ninjaExe"
if ($LASTEXITCODE -ne 0) { throw 'Конфигурация CMake не завершена' }

& $cmakeExe --build $buildDir --parallel 4
if ($LASTEXITCODE -ne 0) { throw 'Сборка не завершена' }

& $ctestExe --test-dir $buildDir --output-on-failure
if ($LASTEXITCODE -ne 0) { throw 'CTest обнаружил ошибку' }
```

Для первой настройки без существующего кэша выберите MinGW toolchain в CLion
или настройте инструменты в `PATH`, затем используйте команды из README.
При смене компилятора создайте новый каталог сборки.

### Другие конфигурации

Адаптеры SimInTech и их тесты добавляются только при `WIN32 AND MINGW`.
Отсутствие этих целей в сборке MSVC, Linux или macOS определяется текущим CMake.
Для многоконфигурационного генератора используйте `--config Debug` при сборке
и `-C Debug` при запуске CTest. Здесь проверена только конфигурация Windows/MinGW,
указанная ниже.

## Проверки

Полный набор зарегистрированных тестов:

```sh
ctest --test-dir out/build/debug -N
ctest --test-dir out/build/debug --output-on-failure
```

После локального изменения интерфейсов можно начать с соответствующей группы:

```sh
ctest --test-dir out/build/debug --output-on-failure -R "^(c_api|perturbed_c_api|perturbed_trajectory_c_api|dll_smoke)$"
ctest --test-dir out/build/debug --output-on-failure -R "^simintech_"
```

Сначала проверьте через `-N`, что выбранные тесты присутствуют в конфигурации.
`dll_smoke` получает путь к собранной библиотеке из CMake и проверяет её
динамическую загрузку. Четыре теста `simintech_*` проверяют адаптеры вне
приложения SimInTech; они не заменяют проверку проекта в самом SimInTech.

### Исходная проверка 15 сентября 2026 года

| Параметр | Проверенное значение |
|---|---|
| Исходный коммит | `390eea2`, с сохранёнными локальными изменениями пользователя |
| Платформа и конфигурация | Windows, MinGW, Ninja, Debug, `BUILD_TESTING=ON` |
| CMake | 4.3.1 |
| GCC | 15.2.0 |
| Ninja | 1.13.2 |
| Результат | **25/25** зарегистрированных тестов прошли |
| C API и загрузка DLL | 3 теста C API и `dll_smoke` прошли |
| Адаптеры SimInTech | 4 теста прошли |

Проверка выполнена в отдельном каталоге `.unlazy/onboarding-20260915/build`:
конфигурация CMake с параметрами выше, `cmake --build` и полный
`ctest --output-on-failure`. Вывод и критерии завершения текущей сессии находятся
в локальном каталоге `.unlazy/onboarding-20260915/`; эти файлы не являются частью
публикуемой документации и могут отсутствовать в другом клоне.

Физические формулы и данные в этой сессии не изменялись. Новые численные
эксперименты, исследования сходимости и независимая проверка по источникам
не проводились. Число прошедших программных тестов не является оценкой
точности аэродинамической модели.

В исходной сборке остались предупреждения компилятора: лишняя точка с запятой
в `third_party/simintech/sit_sparce_interface.h` и игнорируемые результаты
`[[nodiscard]]` в проверках исключений `tests/test_atmosphere.cpp`. Они не
помешали сборке и прохождению тестов; в рамках документационного этапа
эти файлы не изменялись.

## Перед дальнейшей работой с собственной компоновкой

Уточните назначение аппарата, поддерживаемую геометрию и диапазон условий.
Затем составьте перечень входных параметров: какие поля уже используются
адаптером, какие пока служат описанием, какие требуют самостоятельных данных.
Для каждого источника данных и будущей зависимости ведите запись происхождения
в соответствии с `AGENTS.md`. Существующая предварительная модель и наличие
полей геометрии не подтверждают готовность универсального расчёта по компоновке.
