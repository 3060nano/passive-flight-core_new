#include "passive_flight_c_api/PassiveFlightCApi.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace {

#ifdef _WIN32

using ModuleHandle = HMODULE;

/// Загружает динамическую библиотеку средствами Windows.
/// @param path Системный путь к DLL с завершающим нулём.
/// @return Дескриптор модуля либо nullptr при ошибке.
ModuleHandle loadModule(
    const char* path
) {
    return LoadLibraryA(path);
}

/// Освобождает дескриптор Windows-модуля, если он задан.
void unloadModule(
    ModuleHandle module
) {
    if (module != nullptr) {
        FreeLibrary(module);
    }
}

/// Находит экспортированный символ Windows DLL.
/// @return Нетипизированный адрес символа либо nullptr.
void* findSymbol(
    ModuleHandle module,
    const char* symbolName
) {
    return reinterpret_cast<void*>(
        GetProcAddress(
            module,
            symbolName
        )
    );
}

/// Возвращает диагностическое сообщение последней ошибки загрузчика Windows.
std::string moduleErrorMessage() {
    return
        "Windows error code: " +
        std::to_string(
            static_cast<unsigned long>(
                GetLastError()
            )
        );
}

/// Переводит ввод и вывод консоли Windows в кодировку UTF-8.
void configureConsoleEncoding() {
    SetConsoleCP(CP_UTF8);
    SetConsoleOutputCP(CP_UTF8);
}

/// Возвращает штатное имя библиотеки C API для Windows.
const char* defaultLibraryName() {
    return "libpassive_flight_c_api.dll";
}

#else

using ModuleHandle = void*;

/// Загружает динамическую библиотеку средствами POSIX.
/// @param path Системный путь к SO или dylib с завершающим нулём.
/// @return Дескриптор модуля либо nullptr при ошибке.
ModuleHandle loadModule(
    const char* path
) {
    return dlopen(
        path,
        RTLD_NOW
    );
}

/// Освобождает дескриптор POSIX-модуля, если он задан.
void unloadModule(
    ModuleHandle module
) {
    if (module != nullptr) {
        dlclose(module);
    }
}

/// Находит экспортированный символ POSIX-библиотеки.
/// @return Нетипизированный адрес символа либо nullptr.
void* findSymbol(
    ModuleHandle module,
    const char* symbolName
) {
    return dlsym(
        module,
        symbolName
    );
}

/// Возвращает диагностическое сообщение динамического загрузчика POSIX.
std::string moduleErrorMessage() {
    const char* message = dlerror();

    if (message == nullptr) {
        return "Unknown dynamic-loader error";
    }

    return message;
}

/// Не выполняет действий: POSIX-консоль не требует настройки Windows API.
void configureConsoleEncoding() {
}

/// Возвращает штатное имя библиотеки C API для текущей POSIX-платформы.
const char* defaultLibraryName() {
#if defined(__APPLE__)
    return "libpassive_flight_c_api.dylib";
#else
    return "libpassive_flight_c_api.so";
#endif
}

#endif

/// RAII-обёртка над дескриптором динамической библиотеки для дымового теста C API.
class DynamicModule {
public:
    /// Загружает динамическую библиотеку по указанному системному пути.
    /// @param path Путь к DLL, SO или dylib в кодировке, принятой платформой.
    /// @throws std::runtime_error Если модуль невозможно загрузить.
    explicit DynamicModule(
        const std::string& path
    )
        : handle_(
              loadModule(
                  path.c_str()
              )
          ) {
        if (handle_ == nullptr) {
            throw std::runtime_error(
                "Failed to load library: " +
                path +
                "\n" +
                moduleErrorMessage()
            );
        }
    }

    /// Освобождает системный дескриптор загруженной библиотеки.
    ~DynamicModule() {
        unloadModule(handle_);
    }

    /// Запрещает копирование владельца системного дескриптора.
    DynamicModule(
        const DynamicModule&
    ) = delete;

    /// Запрещает копирующее присваивание владельца системного дескриптора.
    DynamicModule& operator=(
        const DynamicModule&
    ) = delete;

    /// Находит экспортированную функцию и преобразует её адрес к заданному типу.
    /// @tparam FunctionType Тип указателя на функцию с требуемым соглашением вызова.
    /// @param functionName ASCII-имя экспортированного символа с завершающим нулём.
    /// @return Типизированный указатель на экспортированную функцию.
    /// @throws std::runtime_error Если символ отсутствует в библиотеке.
    template <typename FunctionType>
    [[nodiscard]]
    FunctionType function(
        const char* functionName
    ) const {
        void* address =
            findSymbol(
                handle_,
                functionName
            );

        if (address == nullptr) {
            throw std::runtime_error(
                "Function was not found in DLL: " +
                std::string(functionName) +
                "\n" +
                moduleErrorMessage()
            );
        }

        return reinterpret_cast<FunctionType>(
            address
        );
    }

private:
    /// Платформенный дескриптор загруженной динамической библиотеки.
    ModuleHandle handle_{};
};

using GetApiVersionFunction =
    const char* (PF_CALL*)(void);

using GetResultCodeNameFunction =
    const char* (PF_CALL*)(int32_t);

using GetObjectCountFunction =
    uint64_t (PF_CALL*)(void);

using GetObjectIdFunction =
    int32_t (PF_CALL*)(
        uint64_t,
        char*,
        uint64_t,
        uint64_t*
    );

using GetObjectDisplayNameFunction =
    int32_t (PF_CALL*)(
        uint64_t,
        char*,
        uint64_t,
        uint64_t*
    );

using CalculateFunction =
    int32_t (PF_CALL*)(
        const PFSimulationInput*,
        PFSimulationOutput*
    );

using CalculateTrajectoryFunction =
    int32_t (PF_CALL*)(
        const PFSimulationInput*,
        PFSimulationOutput*,
        PFTrajectoryPoint*,
        uint64_t,
        uint64_t*,
        uint64_t*
    );

/// Набор обязательных экспортов основного C API, разрешённых из DLL.
struct ApiFunctions {
    /// Функция чтения версии C API.
    GetApiVersionFunction getApiVersion{};
    /// Функция преобразования PFResultCode в текст.
    GetResultCodeNameFunction getResultCodeName{};

    /// Функция чтения числа зарегистрированных объектов.
    GetObjectCountFunction getObjectCount{};
    /// Функция чтения стабильного идентификатора объекта.
    GetObjectIdFunction getObjectId{};
    /// Функция чтения отображаемого имени объекта.
    GetObjectDisplayNameFunction getObjectDisplayName{};

    /// Функция расчёта кратких параметров падения.
    CalculateFunction calculate{};
    /// Функция расчёта кратких параметров и полной траектории.
    CalculateTrajectoryFunction calculateTrajectory{};
};

/// Разрешает все обязательные экспортированные функции C API.
/// @param module Загруженная динамическая библиотека.
/// @return Набор типизированных указателей на функции.
/// @throws std::runtime_error Если хотя бы один экспорт отсутствует.
ApiFunctions loadApiFunctions(
    const DynamicModule& module
) {
    ApiFunctions functions;

    functions.getApiVersion =
        module.function<GetApiVersionFunction>(
            "pfGetApiVersion"
        );

    functions.getResultCodeName =
        module.function<GetResultCodeNameFunction>(
            "pfGetResultCodeName"
        );

    functions.getObjectCount =
        module.function<GetObjectCountFunction>(
            "pfGetObjectCount"
        );

    functions.getObjectId =
        module.function<GetObjectIdFunction>(
            "pfGetObjectId"
        );

    functions.getObjectDisplayName =
        module.function<GetObjectDisplayNameFunction>(
            "pfGetObjectDisplayName"
        );

    functions.calculate =
        module.function<CalculateFunction>(
            "pfCalculate"
        );

    functions.calculateTrajectory =
        module.function<
            CalculateTrajectoryFunction
        >(
            "pfCalculateTrajectory"
        );

    return functions;
}

/// Требует выполнения логического условия дымового теста.
/// @throws std::runtime_error с текстом message, если condition равно false.
void require(
    bool condition,
    const std::string& message
) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

/// Проверяет абсолютную погрешность `abs(actual-expected) <= tolerance`.
/// Все три числовых аргумента задаются в единицах проверяемой величины.
/// @throws std::runtime_error Если условие точности не выполнено.
void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& message
) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message);
    }
}

/// Читает строковое свойство объекта двухпроходным вызовом C API.
/// @param objectIndex Безразмерный индекс объекта в реестре DLL.
/// @param function Функция чтения идентификатора или отображаемого имени.
/// @return Скопированная строка UTF-8.
std::string readString(
    uint64_t objectIndex,
    GetObjectIdFunction function
) {
    uint64_t requiredSize = 0;

    const int32_t queryResult =
        function(
            objectIndex,
            nullptr,
            0,
            &requiredSize
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "String size query failed"
    );

    require(
        requiredSize > 1,
        "Invalid string buffer size"
    );

    std::vector<char> buffer(
        static_cast<std::size_t>(
            requiredSize
        )
    );

    const int32_t readResult =
        function(
            objectIndex,
            buffer.data(),
            requiredSize,
            &requiredSize
        );

    require(
        readResult == PF_RESULT_OK,
        "String reading failed"
    );

    return std::string(
        buffer.data()
    );
}
    
/// Выводит краткие параметры падения с обозначениями единиц СИ.
/// @param output Результат C API: расстояние в м, время в с,
/// скорость в м/с и углы в радианах.
void printSummary(
    const PFSimulationOutput& output
) {
    std::cout
        << "Calculation result:"
        << '\n'
        << "  downrange: "
        << output.downrangeM
        << " m"
        << '\n'
        << "  fall time: "
        << output.fallTimeS
        << " s"
        << '\n'
        << "  impact speed: "
        << output.impactSpeedMps
        << " m/s"
        << '\n'
        << "  impact trajectory angle: "
        << output.impactFlightPathAngleRad
        << " rad"
        << '\n'
        << "  impact pitch angle: "
        << output.impactPitchAngleRad
        << " rad"
        << '\n'
        << "  impact angle of attack: "
        << output.impactAngleOfAttackRad
        << " rad"
        << '\n'
        << "  termination reason: "
        << output.terminationReason
        << '\n';
}

/// Выполняет сквозную проверку загрузки DLL, списка объектов, краткого
/// расчёта и двухпроходного получения траектории.
/// @param libraryPath Путь к динамической библиотеке C API.
/// @throws std::runtime_error При нарушении любого проверяемого контракта.
void runSmokeTest(
    const std::string& libraryPath
) {
    std::cout
        << "Loading DLL: "
        << libraryPath
        << '\n';

    const DynamicModule module(
        libraryPath
    );

    const ApiFunctions api =
        loadApiFunctions(module);

    require(
        api.getApiVersion() != nullptr,
        "API version pointer is null"
    );

    std::cout
        << "C API version: "
        << api.getApiVersion()
        << '\n';

    const uint64_t objectCount =
        api.getObjectCount();

    require(
        objectCount > 0,
        "DLL contains no registered objects"
    );

    std::cout
        << "Registered objects: "
        << objectCount
        << '\n';

    for (uint64_t index = 0;
         index < objectCount;
         ++index) {
        const std::string objectId =
            readString(
                index,
                api.getObjectId
            );

        const std::string displayName =
            readString(
                index,
                api.getObjectDisplayName
            );

        std::cout
            << "  [" << index << "] "
            << objectId
            << " — "
            << displayName
            << '\n';
    }

    const std::string objectId =
        readString(
            0,
            api.getObjectId
        );

    PFSimulationInput input{};

    input.objectId =
        objectId.c_str();

    input.releaseAltitudeM =
        100.0;

    input.releaseSpeedMps =
        200.0;

    PFSimulationOutput summaryOutput{};

    const int32_t summaryResult =
        api.calculate(
            &input,
            &summaryOutput
        );

    if (summaryResult != PF_RESULT_OK) {
        throw std::runtime_error(
            "Summary calculation failed: " +
            std::string(
                api.getResultCodeName(
                    summaryResult
                )
            )
        );
    }

    printSummary(
        summaryOutput
    );

    uint64_t requiredPointCount = 0;
    uint64_t writtenPointCount = 0;

    PFSimulationOutput trajectoryQueryOutput{};

    const int32_t queryResult =
        api.calculateTrajectory(
            &input,
            &trajectoryQueryOutput,
            nullptr,
            0,
            &requiredPointCount,
            &writtenPointCount
        );

    require(
        queryResult ==
            PF_RESULT_BUFFER_TOO_SMALL,
        "Trajectory size query failed"
    );

    require(
        requiredPointCount > 1,
        "Trajectory contains insufficient points"
    );

    std::vector<PFTrajectoryPoint> points(
        static_cast<std::size_t>(
            requiredPointCount
        )
    );

    PFSimulationOutput trajectoryOutput{};

    const int32_t trajectoryResult =
        api.calculateTrajectory(
            &input,
            &trajectoryOutput,
            points.data(),
            requiredPointCount,
            &requiredPointCount,
            &writtenPointCount
        );

    if (trajectoryResult != PF_RESULT_OK) {
        throw std::runtime_error(
            "Trajectory calculation failed: " +
            std::string(
                api.getResultCodeName(
                    trajectoryResult
                )
            )
        );
    }

    require(
        writtenPointCount ==
            requiredPointCount,
        "Incorrect number of written points"
    );

    requireNear(
        points.front().timeS,
        0.0,
        1.0e-12,
        "First trajectory point has incorrect time"
    );

    requireNear(
        points.front().altitudeM,
        input.releaseAltitudeM,
        1.0e-12,
        "First trajectory point has incorrect altitude"
    );

    requireNear(
        points.back().altitudeM,
        0.0,
        1.0e-12,
        "Last trajectory point is not on ground"
    );

    requireNear(
        points.back().downrangeM,
        trajectoryOutput.downrangeM,
        1.0e-9,
        "Trajectory range does not match summary"
    );

    requireNear(
        points.back().timeS,
        trajectoryOutput.fallTimeS,
        1.0e-9,
        "Trajectory time does not match summary"
    );

    requireNear(
        summaryOutput.downrangeM,
        trajectoryOutput.downrangeM,
        1.0e-9,
        "Two DLL calculations produced different range"
    );

    std::cout
        << "Trajectory points: "
        << writtenPointCount
        << '\n'
        << "First point:"
        << '\n'
        << "  t = "
        << points.front().timeS
        << " s"
        << '\n'
        << "  H = "
        << points.front().altitudeM
        << " m"
        << '\n'
        << "Last point:"
        << '\n'
        << "  t = "
        << points.back().timeS
        << " s"
        << '\n'
        << "  x = "
        << points.back().downrangeM
        << " m"
        << '\n'
        << "  H = "
        << points.back().altitudeM
        << " m"
        << '\n';

    std::cout
        << "DLL smoke test passed successfully."
        << '\n';
}

} // namespace

/// Точка входа дымового теста динамической библиотеки.
/// @param argc Число аргументов командной строки.
/// @param argv Массив строк; argv[1] при наличии задаёт путь к библиотеке.
/// @return EXIT_SUCCESS при успешной проверке, иначе EXIT_FAILURE.
int main(
    int argc,
    char* argv[]
) {
    configureConsoleEncoding();

    try {
        const std::string libraryPath =
            argc >= 2
                ? argv[1]
                : defaultLibraryName();

        runSmokeTest(
            libraryPath
        );

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "DLL smoke test failed:"
            << '\n'
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}
